// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Top-Level Modulator
 *
 * Integrates the complete transmitter pipeline:
 *   User data -> BB frame -> BCH -> LDPC -> interleave ->
 *   modulate -> [spread] -> PL frame build -> scramble ->
 *   RRC pulse shape -> baseband IQ output
 */

#include "dvbs2x_vlsnr.h"
#include <stdlib.h>
#include <string.h>

int dvbs2x_modulator_init(struct dvbs2x_modulator *mod,
			  unsigned int modcod_idx,
			  double rolloff,
			  unsigned int sps,
			  unsigned int pl_scrambling_idx)
{
	const struct dvbs2x_modcod *mc;
	int ret;

	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc)
		return -1;

	mod->cfg.modcod = mc;
	mod->cfg.rolloff = rolloff;
	mod->cfg.sps = sps;
	mod->cfg.pl_scrambling_idx = pl_scrambling_idx;

	/* Initialize BB frame context */
	dvbs2x_bb_frame_init(&mod->bb_ctx, mc, DVBS2X_STREAM_TS);

	/* Initialize BCH encoder */
	ret = dvbs2x_bch_encoder_init(&mod->bch_enc, mc);
	if (ret < 0)
		return -1;

	/* Initialize LDPC encoder */
	ret = dvbs2x_ldpc_encoder_init(&mod->ldpc_enc, mc);
	if (ret < 0)
		return -1;

	/* Initialize scrambler */
	dvbs2x_scrambler_init(&mod->scrambler, pl_scrambling_idx);

	/* Initialize TX filter */
	ret = dvbs2x_rrc_filter_init(&mod->tx_filter, rolloff, sps, 16);
	if (ret < 0)
		return -1;

	return 0;
}

int dvbs2x_modulate(struct dvbs2x_modulator *mod,
		    const uint8_t *user_data,
		    unsigned int user_len,
		    struct dvbs2x_complex *output,
		    unsigned int *out_len)
{
	const struct dvbs2x_modcod *mc = mod->cfg.modcod;
	uint8_t *bbframe = NULL;
	uint8_t *bch_out = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *interleaved = NULL;
	struct dvbs2x_complex *data_sym = NULL;
	struct dvbs2x_complex *spread_sym = NULL;
	struct dvbs2x_complex *pl_frame = NULL;
	struct dvbs2x_complex *with_pilots = NULL;
	unsigned int num_data_sym;
	unsigned int num_spread_sym;
	unsigned int pl_frame_len;
	unsigned int pilot_out_len;
	unsigned int rrc_out_len;
	int ret = -1;

	/* Allocate working buffers */
	bbframe = malloc(mc->k_bch);
	bch_out = malloc(mc->n_bch);
	ldpc_out = malloc(mc->fec_len);
	interleaved = malloc(mc->fec_len);

	if (!bbframe || !bch_out || !ldpc_out || !interleaved)
		goto out;

	/* Step 1: Build BB frame */
	if (dvbs2x_bb_frame_build(&mod->bb_ctx, user_data,
				  user_len, bbframe) < 0)
		goto out;

	/* Step 2: BCH encode */
	if (dvbs2x_bch_encode(&mod->bch_enc, bbframe, bch_out) < 0)
		goto out;

	/* Step 3: LDPC encode */
	if (dvbs2x_ldpc_encode(&mod->ldpc_enc, bch_out, ldpc_out) < 0)
		goto out;

	/* Step 4: Interleave */
	dvbs2x_interleave(mc, ldpc_out, interleaved);

	/* Step 5: Modulate */
	if (mc->modulation == DVBS2X_MOD_QPSK)
		num_data_sym = mc->fec_len / 2;
	else
		num_data_sym = mc->fec_len;

	data_sym = malloc(num_data_sym * sizeof(struct dvbs2x_complex));
	if (!data_sym)
		goto out;

	if (mc->modulation == DVBS2X_MOD_QPSK)
		dvbs2x_mod_qpsk(interleaved, data_sym, num_data_sym);
	else
		dvbs2x_mod_pi2bpsk(interleaved, data_sym, num_data_sym);

	/* Step 6: Spread (if applicable) */
	if (mc->has_spread) {
		num_spread_sym = num_data_sym * 2;
		spread_sym = malloc(num_spread_sym *
				    sizeof(struct dvbs2x_complex));
		if (!spread_sym)
			goto out;
		dvbs2x_mod_spread(data_sym, spread_sym, num_data_sym);
	} else {
		num_spread_sym = num_data_sym;
		spread_sym = data_sym;
		data_sym = NULL;	/* prevent double free */
	}

	/* Step 7: Build PL frame (PL header + VL-SNR header + data) */
	pl_frame_len = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
		       num_spread_sym;
	pl_frame = malloc(pl_frame_len * sizeof(struct dvbs2x_complex));
	if (!pl_frame)
		goto out;

	/* PL header */
	dvbs2x_plheader_generate(mc->pls_code, pl_frame);

	/* VL-SNR header */
	dvbs2x_vlsnr_header_generate(mc,
				     pl_frame + DVBS2X_PLHEADER_LEN);

	/* Data symbols */
	memcpy(pl_frame + DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN,
	       spread_sym, num_spread_sym * sizeof(struct dvbs2x_complex));

	/* Step 8: Insert pilots */
	with_pilots = malloc((pl_frame_len + 1000) *
			     sizeof(struct dvbs2x_complex));
	if (!with_pilots)
		goto out;

	/* Copy header portion (no pilots in header) */
	memcpy(with_pilots, pl_frame,
	       (DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN) *
	       sizeof(struct dvbs2x_complex));

	/* Insert pilots in data portion */
	pilot_out_len = dvbs2x_pilot_insert(
		pl_frame + DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN,
		num_spread_sym,
		with_pilots + DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN,
		mc);
	pilot_out_len += DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN;

	/* Step 9: PL scramble (data portion only, skip PL header) */
	dvbs2x_scrambler_reset(&mod->scrambler);
	dvbs2x_scramble(&mod->scrambler,
			with_pilots + DVBS2X_PLHEADER_LEN,
			pilot_out_len - DVBS2X_PLHEADER_LEN);

	/* Step 10: RRC pulse shaping */
	dvbs2x_rrc_filter_reset(&mod->tx_filter);
	dvbs2x_rrc_upsample(&mod->tx_filter, with_pilots, pilot_out_len,
			    output, &rrc_out_len);

	*out_len = rrc_out_len;
	ret = 0;

out:
	free(bbframe);
	free(bch_out);
	free(ldpc_out);
	free(interleaved);
	free(data_sym);
	if (mc->has_spread)
		free(spread_sym);
	free(pl_frame);
	free(with_pilots);
	return ret;
}
