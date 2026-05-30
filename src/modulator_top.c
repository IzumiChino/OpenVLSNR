// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Top-Level Modulator
 *
 * Two layers are provided:
 *
 *   dvbs2x_modulate_symbols() - builds the PL frame at symbol rate
 *       User data -> BB frame -> BCH -> LDPC -> interleave ->
 *       modulate -> [spread] -> PL header + VL-SNR header + pilots ->
 *       scramble (data field only)
 *
 *   dvbs2x_modulate() - wraps the symbol layer with RRC pulse shaping
 *       to produce oversampled baseband IQ.
 *
 * Frame layout (symbols):
 *   [PL header : 90] [VL-SNR header : 896] [scrambled data+pilots]
 *
 * The PL header and VL-SNR (Walsh-Hadamard) header are NOT scrambled
 * so the receiver can correlate against them for frame sync and
 * data-aided carrier recovery.  Only the data field (payload symbols
 * and pilot blocks) is processed by the PL scrambler, matching the
 * DVB-S2 convention.
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

/*
 * Build the PL frame at symbol rate (no pulse shaping).
 * Returns 0 on success; *sym_len receives the number of symbols.
 */
int dvbs2x_modulate_symbols(struct dvbs2x_modulator *mod,
			    const uint8_t *user_data,
			    unsigned int user_len,
			    struct dvbs2x_complex *symbols,
			    unsigned int *sym_len)
{
	const struct dvbs2x_modcod *mc = mod->cfg.modcod;
	uint8_t *bbframe = NULL;
	uint8_t *bch_out = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *interleaved = NULL;
	struct dvbs2x_complex *data_sym = NULL;
	struct dvbs2x_complex *tx_sym = NULL;
	unsigned int num_mod_sym;	/* modulated symbols (pre-spread) */
	unsigned int num_tx_sym;	/* after spreading */
	unsigned int hdr_len;
	unsigned int pilot_len;
	int ret = -1;

	hdr_len = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN;

	bbframe = malloc(mc->k_bch);
	bch_out = malloc(mc->n_bch);
	ldpc_out = malloc(mc->fec_len);
	interleaved = malloc(mc->fec_len);
	if (!bbframe || !bch_out || !ldpc_out || !interleaved)
		goto out;

	/* FEC + interleave */
	if (dvbs2x_bb_frame_build(&mod->bb_ctx, user_data, user_len,
				  bbframe) < 0)
		goto out;
	if (dvbs2x_bch_encode(&mod->bch_enc, bbframe, bch_out) < 0)
		goto out;
	if (dvbs2x_ldpc_encode(&mod->ldpc_enc, bch_out, ldpc_out) < 0)
		goto out;
	dvbs2x_interleave(mc, ldpc_out, interleaved);

	/* Constellation mapping */
	if (mc->modulation == DVBS2X_MOD_QPSK)
		num_mod_sym = mc->fec_len / 2;
	else
		num_mod_sym = mc->fec_len;

	data_sym = malloc(num_mod_sym * sizeof(struct dvbs2x_complex));
	if (!data_sym)
		goto out;

	if (mc->modulation == DVBS2X_MOD_QPSK)
		dvbs2x_mod_qpsk(interleaved, data_sym, num_mod_sym);
	else
		dvbs2x_mod_pi2bpsk(interleaved, data_sym, num_mod_sym);

	/* Spreading */
	if (mc->has_spread) {
		num_tx_sym = num_mod_sym * 2;
		tx_sym = malloc(num_tx_sym * sizeof(struct dvbs2x_complex));
		if (!tx_sym)
			goto out;
		dvbs2x_mod_spread(data_sym, tx_sym, num_mod_sym);
	} else {
		num_tx_sym = num_mod_sym;
		tx_sym = data_sym;
		data_sym = NULL;
	}

	/* Headers (unscrambled) */
	dvbs2x_plheader_generate(mc->pls_code, symbols);
	dvbs2x_vlsnr_header_generate(mc, symbols + DVBS2X_PLHEADER_LEN);

	/* Data field with pilots */
	pilot_len = dvbs2x_pilot_insert(tx_sym, num_tx_sym,
					symbols + hdr_len, mc);

	/* Scramble the data field only (payload + pilots) */
	dvbs2x_scrambler_reset(&mod->scrambler);
	dvbs2x_scramble(&mod->scrambler, symbols + hdr_len, pilot_len);

	*sym_len = hdr_len + pilot_len;
	ret = 0;

out:
	free(bbframe);
	free(bch_out);
	free(ldpc_out);
	free(interleaved);
	free(data_sym);
	free(tx_sym);
	return ret;
}

int dvbs2x_modulate(struct dvbs2x_modulator *mod,
		    const uint8_t *user_data,
		    unsigned int user_len,
		    struct dvbs2x_complex *output,
		    unsigned int *out_len)
{
	const struct dvbs2x_modcod *mc = mod->cfg.modcod;
	struct dvbs2x_complex *frame = NULL;
	unsigned int frame_cap;
	unsigned int frame_len = 0;
	unsigned int rrc_out_len;
	int ret = -1;

	/*
	 * Worst-case frame size: header + all FEC symbols (with spread,
	 * up to 2*fec_len) + one 36-symbol pilot block per 1440 data
	 * symbols, plus slack.
	 */
	frame_cap = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
		    mc->fec_len * 2 +
		    (mc->fec_len * 2 / DVBS2X_PILOT_INTERVAL + 1) *
		    DVBS2X_PILOT_BLK_LEN + 64;

	frame = malloc(frame_cap * sizeof(struct dvbs2x_complex));
	if (!frame)
		goto out;

	if (dvbs2x_modulate_symbols(mod, user_data, user_len,
				    frame, &frame_len) < 0)
		goto out;

	/* RRC pulse shaping (upsample by sps) */
	dvbs2x_rrc_filter_reset(&mod->tx_filter);
	dvbs2x_rrc_upsample(&mod->tx_filter, frame, frame_len,
			    output, &rrc_out_len);

	*out_len = rrc_out_len;
	ret = 0;

out:
	free(frame);
	return ret;
}
