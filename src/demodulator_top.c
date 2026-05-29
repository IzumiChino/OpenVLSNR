// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Top-Level Demodulator
 *
 * Integrates the complete receiver pipeline:
 *   Baseband IQ -> RRC matched filter -> timing recovery ->
 *   coarse freq -> frame sync -> fine freq -> phase est ->
 *   descramble -> pilot extract -> [despread] -> soft demap ->
 *   deinterleave -> LDPC decode -> BCH decode -> BB frame parse
 */

#include "dvbs2x_vlsnr.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Declared in demodulate.c */
extern void dvbs2x_demod_pi2bpsk(const struct dvbs2x_complex *symbols,
				 double *llr, unsigned int len,
				 double noise_var);
extern void dvbs2x_demod_qpsk(const struct dvbs2x_complex *symbols,
			      double *llr, unsigned int num_symbols,
			      double noise_var);
extern void dvbs2x_demod_despread(const struct dvbs2x_complex *in,
				  struct dvbs2x_complex *out,
				  unsigned int out_len);

int dvbs2x_demodulator_init(struct dvbs2x_demodulator *demod,
			    double rolloff,
			    unsigned int sps,
			    unsigned int pl_scrambling_idx)
{
	int ret;

	memset(demod, 0, sizeof(*demod));
	demod->modcod = NULL;	/* determined after frame sync */

	/* Initialize RX matched filter */
	ret = dvbs2x_rrc_filter_init(&demod->rx_filter, rolloff, sps, 16);
	if (ret < 0)
		return -1;

	/* Initialize timing recovery */
	dvbs2x_timing_sync_init(&demod->timing, sps, 1e-4);

	/* Initialize coarse frequency estimator */
	dvbs2x_freq_coarse_init(&demod->freq_coarse, 1e-4);

	/* Initialize fine frequency estimator */
	dvbs2x_freq_fine_init(&demod->freq_fine, 4);

	/* Initialize phase estimator (alpha=0.3 for low SNR) */
	dvbs2x_phase_est_init(&demod->phase, 0.3, 1);

	/* Initialize descrambler */
	dvbs2x_scrambler_init(&demod->descrambler, pl_scrambling_idx);

	demod->sync_frames = 0;
	demod->frame_count = 0;

	return 0;
}

int dvbs2x_demodulate(struct dvbs2x_demodulator *demod,
		      const struct dvbs2x_complex *input,
		      unsigned int in_len,
		      uint8_t *user_data,
		      unsigned int *user_len)
{
	const struct dvbs2x_modcod *mc = NULL;
	struct dvbs2x_complex *filtered = NULL;
	struct dvbs2x_complex *symbols = NULL;
	struct dvbs2x_complex *data_sym = NULL;
	struct dvbs2x_complex *despread = NULL;
	double *llr = NULL;
	double *deinterleaved = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *bch_codeword = NULL;
	unsigned int filt_len;
	unsigned int sym_len;
	unsigned int data_len;
	unsigned int num_sym;
	unsigned int modcod_idx;
	unsigned int frame_offset;
	unsigned int iter_used;
	int ret = -1;

	/* Step 1: Matched filter */
	filtered = malloc(in_len * sizeof(struct dvbs2x_complex));
	if (!filtered)
		goto out;

	dvbs2x_rrc_filter_reset(&demod->rx_filter);
	dvbs2x_rrc_filter_apply(&demod->rx_filter, input, in_len,
				filtered, &filt_len);

	/* Step 2: Symbol timing recovery */
	symbols = malloc((filt_len / demod->timing.sps + 1) *
			 sizeof(struct dvbs2x_complex));
	if (!symbols)
		goto out;

	dvbs2x_timing_sync_process(&demod->timing, filtered, filt_len,
				   symbols, &sym_len);

	/* Step 3: Coarse frequency correction */
	dvbs2x_freq_coarse_process(&demod->freq_coarse, symbols, sym_len);

	/* Step 4: Frame synchronization (WH header correlation) */
	dvbs2x_vlsnr_header_sync(symbols, sym_len,
				 DVBS2X_VLSNR_WH_LEN,
				 &frame_offset, &modcod_idx);

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		goto out;
	demod->modcod = mc;

	/* Skip to frame start (past PL header + VL-SNR header) */
	{
		unsigned int hdr_end;

		hdr_end = frame_offset + DVBS2X_PLHEADER_LEN +
			  DVBS2X_VLSNR_HDR_LEN;
		if (hdr_end >= sym_len)
			goto out;

		/* Step 5: Descramble (skip PL header) */
		dvbs2x_scrambler_reset(&demod->descrambler);
		dvbs2x_descramble(&demod->descrambler,
				  symbols + frame_offset + DVBS2X_PLHEADER_LEN,
				  sym_len - frame_offset - DVBS2X_PLHEADER_LEN);

		/* Step 6: Extract data (remove pilots) */
		if (mc->modulation == DVBS2X_MOD_QPSK)
			num_sym = mc->fec_len / 2;
		else
			num_sym = mc->fec_len;

		if (mc->has_spread)
			num_sym *= 2;

		data_sym = malloc(num_sym * sizeof(struct dvbs2x_complex));
		if (!data_sym)
			goto out;

		data_len = dvbs2x_pilot_extract(
			symbols + hdr_end,
			sym_len - hdr_end,
			data_sym, NULL, mc);

		if (data_len < num_sym)
			num_sym = data_len;
	}

	/* Step 7: Despread if needed */
	if (mc->has_spread) {
		unsigned int despread_len = num_sym / 2;

		despread = malloc(despread_len *
				  sizeof(struct dvbs2x_complex));
		if (!despread)
			goto out;
		dvbs2x_demod_despread(data_sym, despread, despread_len);
		num_sym = despread_len;
	} else {
		despread = data_sym;
		data_sym = NULL;
	}

	/* Step 8: Soft demapping */
	llr = malloc(mc->fec_len * sizeof(double));
	if (!llr)
		goto out;

	{
		/* Estimate noise variance from signal power */
		double noise_var = 0.5;	/* default for low SNR */

		if (mc->modulation == DVBS2X_MOD_QPSK)
			dvbs2x_demod_qpsk(despread, llr, num_sym, noise_var);
		else
			dvbs2x_demod_pi2bpsk(despread, llr, num_sym,
					     noise_var);
	}

	/* Step 9: Deinterleave */
	deinterleaved = malloc(mc->fec_len * sizeof(double));
	if (!deinterleaved)
		goto out;
	dvbs2x_deinterleave(mc, llr, deinterleaved);

	/* Step 10: LDPC decode */
	if (dvbs2x_ldpc_decoder_init(&demod->ldpc_dec, mc,
				     DVBS2X_LDPC_MAX_ITER) < 0)
		goto out;

	ldpc_out = malloc(mc->k_ldpc);
	if (!ldpc_out)
		goto out;

	dvbs2x_ldpc_decode(&demod->ldpc_dec, deinterleaved,
			   ldpc_out, &iter_used);

	/* Step 11: BCH decode */
	if (dvbs2x_bch_decoder_init(&demod->bch_dec, mc) < 0)
		goto out;

	bch_codeword = malloc(mc->n_bch);
	if (!bch_codeword)
		goto out;
	memcpy(bch_codeword, ldpc_out, mc->n_bch);

	if (dvbs2x_bch_decode(&demod->bch_dec, bch_codeword) < 0)
		goto out;

	/* Step 12: BB frame parse */
	dvbs2x_bb_frame_init(&demod->bb_ctx, mc, DVBS2X_STREAM_TS);
	ret = dvbs2x_bb_frame_parse(&demod->bb_ctx, bch_codeword,
				    user_data, user_len);

out:
	free(filtered);
	free(symbols);
	free(data_sym);
	if (mc && mc->has_spread)
		free(despread);
	free(llr);
	free(deinterleaved);
	free(ldpc_out);
	free(bch_codeword);
	return ret;
}
