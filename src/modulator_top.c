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

#define DVBS2X_GOLD_CODE_MAX	262141

int dvbs2x_modulator_init(struct dvbs2x_modulator *mod,
			  unsigned int modcod_idx,
			  double rolloff,
			  unsigned int sps,
			  unsigned int pl_scrambling_idx)
{
	const struct dvbs2x_modcod *mc;
	int ret;

	if (!mod)
		return DVBS2X_ERR_PARAM;
	memset(mod, 0, sizeof(*mod));
	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc || sps < 2 || rolloff < 0.05 || rolloff > 0.35 ||
	    pl_scrambling_idx > DVBS2X_GOLD_CODE_MAX)
		return DVBS2X_ERR_PARAM;

	mod->cfg.modcod = mc;
	mod->cfg.rolloff = rolloff;
	mod->cfg.sps = sps;
	mod->cfg.pl_scrambling_idx = pl_scrambling_idx;

	/* Initialize BB frame context */
	dvbs2x_bb_frame_init(&mod->bb_ctx, mc, DVBS2X_STREAM_TS);

	/* Initialize BCH encoder */
	ret = dvbs2x_bch_encoder_init(&mod->bch_enc, mc);
	if (ret < 0)
		return DVBS2X_ERR_PARAM;

	/* Initialize LDPC encoder */
	ret = dvbs2x_ldpc_encoder_init(&mod->ldpc_enc, mc);
	if (ret < 0)
		return DVBS2X_ERR_PARAM;

	/* Initialize scrambler */
	dvbs2x_scrambler_init(&mod->scrambler, pl_scrambling_idx);

	/* Initialize TX filter */
	ret = dvbs2x_rrc_filter_init(&mod->tx_filter, rolloff, sps, 16);
	if (ret < 0)
		return DVBS2X_ERR_PARAM;

	return 0;
}

void dvbs2x_modulator_destroy(struct dvbs2x_modulator *mod)
{
	if (!mod)
		return;
	/* All modulator state is inline; nothing to free. */
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
	const struct dvbs2x_modcod *mc;
	uint8_t *bbframe = NULL;
	uint8_t *bch_out = NULL;
	uint8_t *ldpc_info = NULL;
	uint8_t *ldpc_cw = NULL;
	uint8_t *tx_bits = NULL;
	uint8_t *interleaved = NULL;
	uint8_t *spread_bits = NULL;
	struct dvbs2x_complex *tx_sym = NULL;
	struct dvbs2x_vlsnr_layout lay = { 0, 0, 0, NULL };
	unsigned int tx_coded;		/* bits after shorten + puncture */
	unsigned int num_tx_sym;
	unsigned int hdr_len;
	unsigned int i, p_idx, t_idx;
	int ret = DVBS2X_ERR_NOMEM;

	if (!sym_len)
		return DVBS2X_ERR_PARAM;
	*sym_len = 0;
	if (!mod || !mod->cfg.modcod || !user_data || !symbols)
		return DVBS2X_ERR_PARAM;
	mc = mod->cfg.modcod;
	if (user_len > mod->bb_ctx.dfl)
		return DVBS2X_ERR_PARAM;

	hdr_len = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN;
	tx_coded = dvbs2x_tx_coded_bits(mc);

	bbframe = malloc(mc->k_bch);
	bch_out = malloc(mc->n_bch);
	ldpc_info = malloc(mc->k_ldpc);
	ldpc_cw = malloc(mc->fec_len);
	tx_bits = malloc(tx_coded);
	interleaved = malloc(tx_coded);
	if (!bbframe || !bch_out || !ldpc_info || !ldpc_cw || !tx_bits ||
	    !interleaved)
		goto out;

	/* Signal the configured roll-off in the BB header MATYPE-1 */
	mod->bb_ctx.ro = dvbs2x_ro_from_rolloff(mod->cfg.rolloff);

	/* BB frame -> BCH -> LDPC */
	if (dvbs2x_bb_frame_build(&mod->bb_ctx, user_data, user_len,
				  bbframe) < 0) {
		ret = DVBS2X_ERR_PARAM;
		goto out;
	}
	if (dvbs2x_bch_encode(&mod->bch_enc, bbframe, bch_out) < 0) {
		ret = DVBS2X_ERR_PARAM;
		goto out;
	}

	/*
	 * Build the LDPC information word and encode it.  Per ETSI EN 302 307
	 * the BCH codeword is prepended with xs shortening zeros (not
	 * transmitted) to fill the k_ldpc-bit LDPC information word.
	 */
	if (mc->xs)
		memset(ldpc_info, 0, mc->xs);
	memcpy(ldpc_info + mc->xs, bch_out, mc->n_bch);
	if (dvbs2x_ldpc_encode(&mod->ldpc_enc, ldpc_info, ldpc_cw) < 0) {
		ret = DVBS2X_ERR_PARAM;
		goto out;
	}

	/*
	 * Apply shortening and puncturing to produce tx_bits:
	 * - Skip first xs info bits (shortened, known zero)
	 * - Copy remaining info bits: ldpc_cw[xs .. k_ldpc-1]
	 * - Copy parity bits, skipping punctured positions
	 */
	t_idx = 0;

	/* Info bits (skip shortened prefix) */
	for (i = mc->xs; i < mc->k_ldpc; i++)
		tx_bits[t_idx++] = ldpc_cw[i];

	/* Parity bits (skip punctured positions) */
	p_idx = 0;
	for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
		if (mc->xp > 0 && p_idx < mc->xp &&
		    i == p_idx * mc->p_period) {
			p_idx++;
			continue;	/* punctured */
		}
		tx_bits[t_idx++] = ldpc_cw[mc->k_ldpc + i];
	}

	/* Interleave the transmitted bits */
	dvbs2x_interleave(mc, tx_bits, interleaved);

	/* Build the data-field layout (data/pilot positions) */
	if (dvbs2x_vlsnr_build_layout(mc, &lay) < 0)
		goto out;

	/*
	 * Constellation mapping.  For SF2 each coded bit is duplicated
	 * (bit-level spreading) before pi/2-BPSK mapping, so the two
	 * copies land on the even and odd diagonals.
	 */
	if (mc->modulation == DVBS2X_MOD_QPSK) {
		num_tx_sym = tx_coded / 2;
		tx_sym = malloc(num_tx_sym * sizeof(struct dvbs2x_complex));
		if (!tx_sym)
			goto out;
		dvbs2x_mod_qpsk(interleaved, tx_sym, num_tx_sym);
	} else if (mc->has_spread) {
		num_tx_sym = tx_coded * 2;
		spread_bits = malloc(num_tx_sym);
		tx_sym = malloc(num_tx_sym * sizeof(struct dvbs2x_complex));
		if (!spread_bits || !tx_sym)
			goto out;
		dvbs2x_mod_spread_bits(interleaved, spread_bits, tx_coded);
		dvbs2x_mod_pi2bpsk(spread_bits, tx_sym, num_tx_sym);
	} else {
		num_tx_sym = tx_coded;
		tx_sym = malloc(num_tx_sym * sizeof(struct dvbs2x_complex));
		if (!tx_sym)
			goto out;
		dvbs2x_mod_pi2bpsk(interleaved, tx_sym, num_tx_sym);
	}

	if (num_tx_sym != lay.num_data) {
		ret = DVBS2X_ERR_PARAM;
		goto out;
	}

	/* Headers (unscrambled) */
	dvbs2x_plheader_generate(mc->pls_code, symbols);
	dvbs2x_vlsnr_header_generate(mc, symbols + DVBS2X_PLHEADER_LEN);

	/* Data field: interleave payload with pilots per the layout */
	dvbs2x_pilot_insert(tx_sym, &lay, symbols + hdr_len);

	/*
	 * Scramble the data field (payload + pilots).  The VL-SNR data
	 * field reads the PL scrambler from index VLSNR_HDR_LEN (900);
	 * non-QPSK payload is scrambled by real +/-1, pilots by 4-phase.
	 */
	dvbs2x_scrambler_reset(&mod->scrambler);
	dvbs2x_scrambler_seek(&mod->scrambler, DVBS2X_VLSNR_HDR_LEN);
	dvbs2x_scramble_field(&mod->scrambler, symbols + hdr_len,
			      lay.field_len, lay.is_pilot,
			      mc->modulation == DVBS2X_MOD_QPSK);

	*sym_len = hdr_len + lay.field_len;
	ret = DVBS2X_OK;

out:
	free(bbframe);
	free(bch_out);
	free(ldpc_info);
	free(ldpc_cw);
	free(tx_bits);
	free(interleaved);
	free(spread_bits);
	free(tx_sym);
	dvbs2x_vlsnr_free_layout(&lay);
	return ret;
}

int dvbs2x_modulate(struct dvbs2x_modulator *mod,
		    const uint8_t *user_data,
		    unsigned int user_len,
		    struct dvbs2x_complex *output,
		    unsigned int *out_len)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *frame = NULL;
	unsigned int frame_cap;
	unsigned int frame_len = 0;
	unsigned int rrc_out_len;
	int ret = DVBS2X_ERR_NOMEM;

	if (!out_len)
		return DVBS2X_ERR_PARAM;
	*out_len = 0;
	if (!mod || !mod->cfg.modcod || !user_data || !output)
		return DVBS2X_ERR_PARAM;
	mc = mod->cfg.modcod;
	if (user_len > mod->bb_ctx.dfl)
		return DVBS2X_ERR_PARAM;

	/*
	 * Worst-case frame size: header + all transmitted symbols
	 * (with spread, up to 2*tx_coded) + pilot blocks + slack.  The
	 * VL-SNR pilot overhead is well under 2*tx_coded/4.
	 */
	frame_cap = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
		    dvbs2x_tx_coded_bits(mc) * 2 +
		    dvbs2x_tx_coded_bits(mc) / 2 + 128;

	frame = malloc(frame_cap * sizeof(struct dvbs2x_complex));
	if (!frame)
		goto out;

	ret = dvbs2x_modulate_symbols(mod, user_data, user_len,
				      frame, &frame_len);
	if (ret < 0)
		goto out;

	/* RRC pulse shaping (upsample by sps) */
	dvbs2x_rrc_filter_reset(&mod->tx_filter);
	dvbs2x_rrc_upsample(&mod->tx_filter, frame, frame_len,
			    output, &rrc_out_len);

	*out_len = rrc_out_len;
	ret = DVBS2X_OK;

out:
	free(frame);
	return ret;
}
