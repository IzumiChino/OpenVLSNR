// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Top-Level Demodulator
 *
 * Two layers are provided:
 *
 *   dvbs2x_demodulate_symbols() - symbol-rate receiver core
 *       frame sync -> WH-aided carrier recovery -> descramble ->
 *       pilot-aided phase tracking -> pilot extract -> [despread] ->
 *       soft demap -> deinterleave -> LDPC decode -> BCH decode ->
 *       BB frame parse
 *
 *   dvbs2x_demodulate() - wraps the symbol core with the RF front-end
 *       (RRC matched filter + symbol timing recovery).
 *
 * Carrier recovery exploits the known 896-symbol Walsh-Hadamard
 * header and the periodic pilot blocks:
 *
 *   - When the frame-sync correlation is strong (higher SNR) a
 *     coarse L&R frequency estimate from the header removes large
 *     carrier offsets.
 *   - A weighted, unwrapped linear fit of the phase measured on the
 *     header and every pilot block then tracks the residual carrier
 *     (small frequency drift + phase) across the whole frame.  The
 *     header (896 symbols) anchors the phase and the pilots, spread
 *     over the frame, give the long baseline needed for an accurate
 *     residual-frequency slope.  This is what lets the data field be
 *     coherently demapped down to very low Es/N0.
 */

#include "dvbs2x_vlsnr.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M_SQRT1_2_D	0.70710678118654752440

/*
 * L&R coarse-frequency lags, and the minimum estimated Es/N0 (dB) at
 * which the header-only coarse frequency estimate is trustworthy.
 * Below this the receiver assumes a small residual offset (as after
 * acquisition) and relies on pilot-aided residual tracking, which is
 * what allows operation at very low Es/N0.
 */
#define LR_LAGS		128
#define COARSE_MIN_ESN0	4.0

/*
 * Frame-sync correlation segment length.  The 896-symbol header is
 * correlated as coherent segments of this length whose magnitudes are
 * summed, which tolerates a residual carrier offset up to about
 * 0.25/SYNC_SEG_LEN cycles/symbol during acquisition while keeping
 * enough per-segment gain to discriminate the MODCOD at low SNR.
 */
#define SYNC_SEG_LEN	128

/*
 * Frame-acquisition search window (symbols).  The demodulator expects
 * the frame to start within this many symbols of the buffer start, as
 * a receiver would after coarse burst detection.  Bounding the search
 * keeps acquisition cost independent of the buffer length.
 */
#define ACQ_WINDOW	4096

/*
 * Number of pilot-bearing data symbols (after spreading) and the
 * resulting data-field length including 36-symbol pilot blocks.
 */
static void frame_geometry(const struct dvbs2x_modcod *mc,
			   unsigned int *num_tx_sym,
			   unsigned int *data_field_len)
{
	unsigned int tx_coded;
	unsigned int mod_sym;
	unsigned int tx_sym;
	unsigned int blks;

	tx_coded = dvbs2x_tx_coded_bits(mc);

	if (mc->modulation == DVBS2X_MOD_QPSK)
		mod_sym = tx_coded / 2;
	else
		mod_sym = tx_coded;

	tx_sym = mc->has_spread ? mod_sym * 2 : mod_sym;
	blks = (tx_sym - 1) / DVBS2X_PILOT_INTERVAL;

	*num_tx_sym = tx_sym;
	*data_field_len = tx_sym + blks * DVBS2X_PILOT_BLK_LEN;
}

/* Rotate symbols [base, base+span) by exp(-j(2*pi*f*n + phi)), n from 0. */
static void derotate(struct dvbs2x_complex *sym, unsigned int base,
		     unsigned int span, double f, double phi)
{
	unsigned int n;

	for (n = 0; n < span; n++) {
		struct dvbs2x_complex *r = &sym[base + n];
		double a = 2.0 * M_PI * f * (double)n + phi;
		double c = cos(a), s = sin(a);
		double ti = r->i * c + r->q * s;
		double tq = r->q * c - r->i * s;

		r->i = ti;
		r->q = tq;
	}
}

/*
 * L&R frequency estimate (cycles/symbol) of z[0..N-1].
 * Estimation range is +/- 1/(LR_LAGS+1); apply a coarse single-lag
 * pre-estimate first to widen the capture range.
 */
static double lr_freq(const struct dvbs2x_complex *z, unsigned int n)
{
	double si = 0.0, sq = 0.0;
	unsigned int m, k;

	for (m = 1; m <= LR_LAGS && m < n; m++) {
		double ri = 0.0, rq = 0.0;

		for (k = m; k < n; k++) {
			ri += z[k].i * z[k - m].i + z[k].q * z[k - m].q;
			rq += z[k].q * z[k - m].i - z[k].i * z[k - m].q;
		}
		si += ri / (double)(n - m);
		sq += rq / (double)(n - m);
	}
	return atan2(sq, si) / (M_PI * (double)(LR_LAGS + 1));
}

/*
 * Estimate the per-component noise variance from the WH header using
 * second differences of z[n] = r * conj(ref).  The second difference
 * cancels a constant carrier-frequency ramp, so the estimate is robust
 * to a residual offset.  E|z[n]-2z[n-1]+z[n-2]|^2 = 12*sigma^2.
 */
static double estimate_noise_var(const struct dvbs2x_complex *sym,
				 unsigned int wh_start,
				 const struct dvbs2x_complex *wh_ref)
{
	double acc = 0.0;
	unsigned int n, cnt = 0;
	struct dvbs2x_complex z0, z1, z2;

	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		z2.i = r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		z2.q = r->q * wh_ref[n].i - r->i * wh_ref[n].q;
		if (n >= 2) {
			double di = z2.i - 2.0 * z1.i + z0.i;
			double dq = z2.q - 2.0 * z1.q + z0.q;

			acc += di * di + dq * dq;
			cnt++;
		}
		z0 = z1;
		z1 = z2;
	}
	if (cnt == 0)
		return 1.0;
	return acc / (double)cnt / 12.0;
}

/*
 * Coarse carrier-frequency recovery from the WH header.  Only applied
 * when the estimated Es/N0 is high enough to trust a header-only
 * estimate (large offsets are otherwise left to acquisition at higher
 * SNR; small residual offsets are handled by residual tracking).
 */
static void coarse_freq_recover(struct dvbs2x_complex *sym,
				unsigned int wh_start, unsigned int span,
				const struct dvbs2x_complex *wh_ref,
				double esn0_db)
{
	struct dvbs2x_complex *z;
	double f1, f2;
	double ri = 0.0, rq = 0.0;
	unsigned int n;

	if (esn0_db < COARSE_MIN_ESN0)
		return;

	z = malloc(DVBS2X_VLSNR_WH_LEN * sizeof(*z));
	if (!z)
		return;

	/* z[n] = r * conj(ref) */
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		z[n].i = r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		z[n].q = r->q * wh_ref[n].i - r->i * wh_ref[n].q;
	}

	/* Single-lag coarse estimate (wide range) */
	for (n = 1; n < DVBS2X_VLSNR_WH_LEN; n++) {
		ri += z[n].i * z[n - 1].i + z[n].q * z[n - 1].q;
		rq += z[n].q * z[n - 1].i - z[n].i * z[n - 1].q;
	}
	f1 = atan2(rq, ri) / (2.0 * M_PI);

	/* Remove f1 from z, then refine with L&R */
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		double a = -2.0 * M_PI * f1 * (double)n;
		double c = cos(a), s = sin(a);
		double ti = z[n].i * c - z[n].q * s;
		double tq = z[n].i * s + z[n].q * c;

		z[n].i = ti;
		z[n].q = tq;
	}
	f2 = lr_freq(z, DVBS2X_VLSNR_WH_LEN);

	derotate(sym, wh_start, span, f1 + f2, 0.0);
	free(z);
}

/*
 * Residual carrier tracking: measure the phase on the header and on
 * every pilot block, unwrap, weighted-linear-fit phase against frame
 * position, and de-rotate the whole data field.
 */
static void residual_carrier_track(struct dvbs2x_complex *sym,
				   unsigned int wh_start,
				   unsigned int data_field_len,
				   unsigned int num_tx_sym,
				   const struct dvbs2x_complex *wh_ref)
{
	double pos[DVBS2X_VLSNR_FRAME_LONG / DVBS2X_PILOT_INTERVAL + 4];
	double ph[DVBS2X_VLSNR_FRAME_LONG / DVBS2X_PILOT_INTERVAL + 4];
	double wt[DVBS2X_VLSNR_FRAME_LONG / DVBS2X_PILOT_INTERVAL + 4];
	unsigned int ns = 0;
	unsigned int data_start = wh_start + DVBS2X_VLSNR_WH_LEN + 2;
	unsigned int blk_off, n;
	double ci, cq;
	double sw, swp, swt, swpp, swpt, denom, a, b;
	unsigned int k;

	/* Header phase sample (anchor) at the header centre */
	ci = 0.0;
	cq = 0.0;
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		ci += r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		cq += r->q * wh_ref[n].i - r->i * wh_ref[n].q;
	}
	pos[ns] = (double)(DVBS2X_VLSNR_WH_LEN - 1) / 2.0;
	ph[ns] = atan2(cq, ci);
	wt[ns] = (double)DVBS2X_VLSNR_WH_LEN;
	ns++;

	/* One phase sample per pilot block (vs reference (1+j)/sqrt2) */
	for (blk_off = DVBS2X_PILOT_INTERVAL;
	     blk_off + DVBS2X_PILOT_BLK_LEN <= data_field_len;
	     blk_off += DVBS2X_PILOT_INTERVAL + DVBS2X_PILOT_BLK_LEN) {
		ci = 0.0;
		cq = 0.0;
		for (n = 0; n < DVBS2X_PILOT_BLK_LEN; n++) {
			const struct dvbs2x_complex *r =
				&sym[data_start + blk_off + n];

			ci += (r->i + r->q) * M_SQRT1_2_D;
			cq += (r->q - r->i) * M_SQRT1_2_D;
		}
		pos[ns] = (double)DVBS2X_VLSNR_WH_LEN + (double)blk_off +
			  (double)(DVBS2X_PILOT_BLK_LEN - 1) / 2.0;
		ph[ns] = atan2(cq, ci);
		wt[ns] = (double)DVBS2X_PILOT_BLK_LEN;
		ns++;
	}

	/* Unwrap phase samples (ordered by increasing position) */
	for (k = 1; k < ns; k++) {
		double d = ph[k] - ph[k - 1];

		while (d > M_PI) {
			ph[k] -= 2.0 * M_PI;
			d = ph[k] - ph[k - 1];
		}
		while (d < -M_PI) {
			ph[k] += 2.0 * M_PI;
			d = ph[k] - ph[k - 1];
		}
	}

	/* Weighted linear fit phase = a*pos + b */
	sw = swp = swt = swpp = swpt = 0.0;
	for (k = 0; k < ns; k++) {
		sw += wt[k];
		swp += wt[k] * pos[k];
		swt += wt[k] * ph[k];
		swpp += wt[k] * pos[k] * pos[k];
		swpt += wt[k] * pos[k] * ph[k];
	}
	denom = sw * swpp - swp * swp;
	if (fabs(denom) < 1e-9) {
		a = 0.0;
		b = swt / sw;
	} else {
		a = (sw * swpt - swp * swt) / denom;
		b = (swt - a * swp) / sw;
	}

	/* De-rotate the data field by the fitted phase line */
	for (n = 0; n < data_field_len; n++) {
		struct dvbs2x_complex *r = &sym[data_start + n];
		double angle = a * (double)(DVBS2X_VLSNR_WH_LEN + n) + b;
		double c = cos(angle), s = sin(angle);
		double ti = r->i * c + r->q * s;
		double tq = r->q * c - r->i * s;

		r->i = ti;
		r->q = tq;
	}

	(void)num_tx_sym;
}

int dvbs2x_demodulator_init(struct dvbs2x_demodulator *demod,
			    double rolloff,
			    unsigned int sps,
			    unsigned int pl_scrambling_idx)
{
	memset(demod, 0, sizeof(*demod));
	demod->modcod = NULL;	/* determined after frame sync */

	if (sps < 2)
		return DVBS2X_ERR_PARAM;

	if (dvbs2x_rrc_filter_init(&demod->rx_filter, rolloff, sps, 16) < 0)
		return DVBS2X_ERR_PARAM;

	dvbs2x_timing_sync_init(&demod->timing, sps, 1e-3);
	dvbs2x_freq_coarse_init(&demod->freq_coarse, 1e-4);
	dvbs2x_freq_fine_init(&demod->freq_fine, 4);
	dvbs2x_phase_est_init(&demod->phase, 0.3, 1);
	dvbs2x_afc_init(&demod->afc, 0.95);
	dvbs2x_scrambler_init(&demod->descrambler, pl_scrambling_idx);

	demod->sync_frames = 0;
	demod->frame_count = 0;

	return 0;
}

void dvbs2x_demodulator_destroy(struct dvbs2x_demodulator *demod)
{
	dvbs2x_ldpc_decoder_free(&demod->ldpc_dec);
}

int dvbs2x_demodulate_symbols(struct dvbs2x_demodulator *demod,
			      const struct dvbs2x_complex *input,
			      unsigned int in_len,
			      double noise_var,
			      uint8_t *user_data,
			      unsigned int *user_len)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *work = NULL;
	struct dvbs2x_complex *wh_ref = NULL;
	struct dvbs2x_complex *data_sym = NULL;
	struct dvbs2x_complex *pilots = NULL;
	struct dvbs2x_complex *despread = NULL;
	double *llr = NULL;
	double *deint = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *bch_cw = NULL;
	unsigned int wh_start = 0, modcod_idx = 0;
	unsigned int search_len;
	unsigned int num_tx_sym, data_field_len, data_start;
	unsigned int ndata, num_mod_sym;
	unsigned int iter_used;
	double conf, demap_nv, nv_est, esn0_db;
	int ret = DVBS2X_ERR_FEC;

	if (in_len < DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_WH_LEN)
		return DVBS2X_ERR_SHORT;

	/* Mutable copy: carrier recovery / descrambling work in place */
	work = malloc(in_len * sizeof(struct dvbs2x_complex));
	if (!work)
		return DVBS2X_ERR_NOMEM;
	memcpy(work, input, in_len * sizeof(struct dvbs2x_complex));

	/*
	 * AFC pre-correction: only in streaming mode (ACQUIRE/TRACK).
	 * In burst mode (state == SEARCH), each frame is independent
	 * and the per-frame coarse estimator handles the offset.
	 */
	if (demod->state != DVBS2X_DEMOD_SEARCH && demod->afc.freq_est != 0.0)
		derotate(work, 0, in_len, demod->afc.freq_est, 0.0);

	/*
	 * Frame sync via Walsh-Hadamard correlation, bounded to the
	 * acquisition window (the header read still extends past it).
	 */
	search_len = in_len;
	if (search_len > ACQ_WINDOW + DVBS2X_VLSNR_WH_LEN)
		search_len = ACQ_WINDOW + DVBS2X_VLSNR_WH_LEN;
	conf = dvbs2x_vlsnr_header_sync(work, search_len, SYNC_SEG_LEN,
					&wh_start, &modcod_idx);
	if (conf < 0.05) {
		ret = DVBS2X_ERR_NOSYNC;
		goto out;
	}
	mc = dvbs2x_vlsnr_get_modcod(modcod_idx);
	if (!mc) {
		ret = DVBS2X_ERR_NOSYNC;
		goto out;
	}
	demod->modcod = mc;

	frame_geometry(mc, &num_tx_sym, &data_field_len);
	data_start = wh_start + DVBS2X_VLSNR_WH_LEN + 2;
	if (getenv("DVBS2X_DEBUG"))
		fprintf(stderr,
			"[dbg] wh_start=%u modcod=%u conf=%.3f tx_sym=%u "
			"dfl=%u in_len=%u\n",
			wh_start, modcod_idx, conf, num_tx_sym,
			data_field_len, in_len);
	if (data_start + data_field_len > in_len) {
		ret = DVBS2X_ERR_SHORT;
		goto out;
	}

	/* Carrier recovery */
	wh_ref = malloc(DVBS2X_VLSNR_HDR_LEN * sizeof(struct dvbs2x_complex));
	if (!wh_ref) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	dvbs2x_vlsnr_header_generate(mc, wh_ref);

	/* Estimate noise/SNR from the header (robust to small offsets) */
	nv_est = estimate_noise_var(work, wh_start, wh_ref + 2);
	esn0_db = 10.0 * log10(1.0 / (2.0 * nv_est + 1e-12));
	if (getenv("DVBS2X_DEBUG"))
		fprintf(stderr, "[dbg] nv_est=%.4f esn0_est=%.2f dB\n",
			nv_est, esn0_db);

	/*
	 * Multi-frame AFC: update the coherent single-lag accumulator
	 * from this frame's header.  Only active in streaming mode;
	 * in burst mode the state remains SEARCH and the AFC estimate
	 * is not applied as pre-correction.
	 */
	if (demod->state != DVBS2X_DEMOD_SEARCH)
		dvbs2x_afc_update(&demod->afc, work, wh_start,
				  wh_ref + 2);

	/*
	 * Single-frame coarse frequency correction (L&R): only applied
	 * at high SNR where a single header gives a reliable estimate.
	 * At low SNR the AFC pre-correction + residual tracker suffice.
	 */
	coarse_freq_recover(work, wh_start,
			    DVBS2X_VLSNR_WH_LEN + 2 + data_field_len,
			    wh_ref + 2, esn0_db);

	/* Descramble the data field (payload + pilots) */
	dvbs2x_scrambler_reset(&demod->descrambler);
	dvbs2x_descramble(&demod->descrambler, work + data_start,
			  data_field_len);

	/* Residual phase/frequency tracking from header + pilots */
	residual_carrier_track(work, wh_start, data_field_len,
			       num_tx_sym, wh_ref + 2);

	/* Extract pilots and data symbols */
	data_sym = malloc(num_tx_sym * sizeof(struct dvbs2x_complex));
	pilots = malloc((data_field_len - num_tx_sym + 1) *
			sizeof(struct dvbs2x_complex));
	if (!data_sym || !pilots) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}

	ndata = dvbs2x_pilot_extract(work + data_start, data_field_len,
				     data_sym, pilots, mc);

	/* Despread if needed */
	if (mc->has_spread) {
		unsigned int dl = ndata / 2;

		despread = malloc(dl * sizeof(struct dvbs2x_complex));
		if (!despread) {
			ret = DVBS2X_ERR_NOMEM;
			goto out;
		}
		dvbs2x_demod_despread(data_sym, despread, dl);
		num_mod_sym = dl;
	} else {
		despread = data_sym;
		data_sym = NULL;
		num_mod_sym = ndata;
	}

	/* Soft demapping */
	demap_nv = (noise_var > 0.0) ? noise_var : nv_est;
	if (mc->has_spread)
		demap_nv *= 0.5;	/* despread halves the noise */

	{
		unsigned int tx_coded = dvbs2x_tx_coded_bits(mc);

		llr = malloc(tx_coded * sizeof(double));
		deint = malloc(mc->fec_len * sizeof(double));
		if (!llr || !deint) {
			ret = DVBS2X_ERR_NOMEM;
			goto out;
		}

		if (mc->modulation == DVBS2X_MOD_QPSK)
			dvbs2x_demod_qpsk(despread, llr,
					  num_mod_sym, demap_nv);
		else
			dvbs2x_demod_pi2bpsk(despread, llr,
					     num_mod_sym, demap_nv);

		/* Deinterleave (operates on tx_coded bits) */
		dvbs2x_deinterleave(mc, llr, deint);

		/*
		 * Depuncture and deshorten: expand tx_coded LLRs
		 * back to the full fec_len LDPC codeword.
		 *
		 * deint[0 .. k_ldpc-xs-1] = info LLRs
		 * deint[k_ldpc-xs .. tx_coded-1] = unpunctured parity
		 *
		 * Rebuild full_llr[0..fec_len-1]:
		 *   [0..xs-1] = +LLR_CLAMP (shortened, known zero)
		 *   [xs..k_ldpc-1] = info LLRs from deint
		 *   [k_ldpc..fec_len-1] = parity with 0.0 at
		 *                         punctured positions
		 */
		{
			double *full_llr;
			unsigned int i, p_idx, d_idx;

			full_llr = malloc(mc->fec_len * sizeof(double));
			if (!full_llr) {
				ret = DVBS2X_ERR_NOMEM;
				goto out;
			}

			/* Shortened positions: known zero */
			for (i = 0; i < mc->xs; i++)
				full_llr[i] = 40.0;

			/* Info LLRs */
			for (i = mc->xs; i < mc->k_ldpc; i++)
				full_llr[i] = deint[i - mc->xs];

			/* Parity: insert erasures at punctured pos */
			d_idx = mc->k_ldpc - mc->xs;
			p_idx = 0;
			for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
				if (mc->xp > 0 && p_idx < mc->xp &&
				    i == p_idx * mc->p_period) {
					full_llr[mc->k_ldpc + i] = 0.0;
					p_idx++;
				} else {
					full_llr[mc->k_ldpc + i] =
						deint[d_idx++];
				}
			}

			/* Replace deint with full_llr for LDPC */
			free(deint);
			deint = full_llr;
		}
	}

	/* LDPC decode */
	if (!demod->ldpc_dec.csr_row_ptr ||
	    demod->ldpc_dec.code.n != mc->fec_len ||
	    demod->ldpc_dec.code.k != mc->k_ldpc) {
		dvbs2x_ldpc_decoder_free(&demod->ldpc_dec);
		if (dvbs2x_ldpc_decoder_init(&demod->ldpc_dec, mc,
					     DVBS2X_LDPC_MAX_ITER) < 0) {
			ret = DVBS2X_ERR_NOMEM;
			goto out;
		}
	}
	ldpc_out = malloc(mc->k_ldpc);
	if (!ldpc_out) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	dvbs2x_ldpc_decode(&demod->ldpc_dec, deint, ldpc_out, &iter_used);

	/*
	 * BCH decode: the LDPC info word includes the xs zero prefix.
	 * The full k_ldpc = n_bch bits form the BCH codeword.
	 */
	if (dvbs2x_bch_decoder_init(&demod->bch_dec, mc) < 0) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	bch_cw = malloc(mc->n_bch);
	if (!bch_cw) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	memcpy(bch_cw, ldpc_out, mc->n_bch);
	if (dvbs2x_bch_decode(&demod->bch_dec, bch_cw) < 0) {
		ret = DVBS2X_ERR_CRC;
		goto out;
	}

	/* BB frame parse */
	dvbs2x_bb_frame_init(&demod->bb_ctx, mc, DVBS2X_STREAM_TS);
	if (dvbs2x_bb_frame_parse(&demod->bb_ctx, bch_cw,
				  user_data, user_len) < 0)
		ret = DVBS2X_ERR_CRC;
	else
		ret = DVBS2X_OK;

out:
	free(work);
	free(wh_ref);
	free(data_sym);
	free(pilots);
	free(despread);
	free(llr);
	free(deint);
	free(ldpc_out);
	free(bch_cw);
	return ret;
}

int dvbs2x_demodulate(struct dvbs2x_demodulator *demod,
		      const struct dvbs2x_complex *input,
		      unsigned int in_len,
		      uint8_t *user_data,
		      unsigned int *user_len)
{
	struct dvbs2x_complex *filtered = NULL;
	struct dvbs2x_complex *symbols = NULL;
	unsigned int filt_len = 0, sym_len = 0;
	int ret = DVBS2X_ERR_NOMEM;

	/* Matched filter */
	filtered = malloc(in_len * sizeof(struct dvbs2x_complex));
	if (!filtered)
		goto out;
	dvbs2x_rrc_filter_reset(&demod->rx_filter);
	dvbs2x_rrc_filter_apply(&demod->rx_filter, input, in_len,
				filtered, &filt_len);

	/* Symbol timing recovery */
	symbols = malloc((filt_len / demod->timing.sps + 2) *
			 sizeof(struct dvbs2x_complex));
	if (!symbols)
		goto out;
	dvbs2x_timing_sync_init(&demod->timing, demod->timing.sps, 1e-3);
	dvbs2x_timing_sync_process(&demod->timing, filtered, filt_len,
				   symbols, &sym_len);

	ret = dvbs2x_demodulate_symbols(demod, symbols, sym_len, 0.0,
					user_data, user_len);

out:
	free(filtered);
	free(symbols);
	return ret;
}

/*
 * Lock management thresholds: consecutive successes to declare TRACK,
 * consecutive failures to revert to SEARCH.
 */
#define LOCK_THRESH	3
#define LOSS_THRESH	3

int dvbs2x_demodulate_stream(struct dvbs2x_demodulator *demod,
			     const struct dvbs2x_complex *input,
			     unsigned int in_len,
			     uint8_t *user_data,
			     unsigned int *user_len,
			     unsigned int *consumed)
{
	struct dvbs2x_complex *filtered = NULL;
	struct dvbs2x_complex *symbols = NULL;
	unsigned int filt_len = 0, sym_len = 0;
	unsigned int sps = demod->timing.sps;
	unsigned int need_samples;
	unsigned int feed_len;
	int ret;

	*consumed = 0;
	*user_len = 0;

	/*
	 * Estimate samples needed for one frame.  In TRACK mode use
	 * the known frame length; in SEARCH/ACQUIRE use the largest
	 * possible VL-SNR frame plus acquisition margin.
	 */
	if (demod->state == DVBS2X_DEMOD_TRACK &&
	    demod->expected_frame_len > 0)
		need_samples = (demod->expected_frame_len + 128) * sps;
	else
		need_samples = (DVBS2X_VLSNR_FRAME_LONG +
				DVBS2X_PLHEADER_LEN +
				DVBS2X_VLSNR_HDR_LEN + ACQ_WINDOW) * sps;

	if (in_len < need_samples) {
		*consumed = 0;
		return DVBS2X_ERR_SHORT;
	}

	/*
	 * Feed only one frame's worth of samples to the filter to
	 * avoid processing future frames prematurely.  Add margin
	 * for the acquisition window and filter transient.
	 */
	feed_len = need_samples;
	if (feed_len > in_len)
		feed_len = in_len;

	/* Matched filter (persistent state across calls) */
	filtered = malloc(feed_len * sizeof(struct dvbs2x_complex));
	if (!filtered)
		return DVBS2X_ERR_NOMEM;

	if (!demod->filter_primed) {
		dvbs2x_rrc_filter_reset(&demod->rx_filter);
		demod->filter_primed = 1;
	}
	dvbs2x_rrc_filter_apply(&demod->rx_filter, input, feed_len,
				filtered, &filt_len);

	/* Symbol timing recovery (persistent mu across calls) */
	symbols = malloc((filt_len / sps + 2) *
			 sizeof(struct dvbs2x_complex));
	if (!symbols) {
		free(filtered);
		return DVBS2X_ERR_NOMEM;
	}
	dvbs2x_timing_sync_process(&demod->timing, filtered, filt_len,
				   symbols, &sym_len);
	free(filtered);

	/* Attempt frame decode at symbol level */
	ret = dvbs2x_demodulate_symbols(demod, symbols, sym_len, 0.0,
					user_data, user_len);
	free(symbols);

	/* Update state machine */
	if (ret == 0) {
		demod->sync_frames++;
		demod->frame_count++;

		/* Compute expected frame length for next prediction */
		if (demod->modcod) {
			unsigned int tx_sym, dfl;

			frame_geometry(demod->modcod, &tx_sym, &dfl);
			demod->expected_frame_len =
				DVBS2X_PLHEADER_LEN +
				DVBS2X_VLSNR_HDR_LEN + dfl;
		}

		/* State transitions on success */
		if (demod->state == DVBS2X_DEMOD_SEARCH)
			demod->state = DVBS2X_DEMOD_ACQUIRE;
		if (demod->state == DVBS2X_DEMOD_ACQUIRE &&
		    demod->sync_frames >= LOCK_THRESH)
			demod->state = DVBS2X_DEMOD_TRACK;
	} else {
		demod->sync_frames = 0;

		/* Revert to SEARCH after consecutive failures */
		if (demod->state == DVBS2X_DEMOD_TRACK) {
			demod->frame_count++;
			if (demod->frame_count >= LOSS_THRESH) {
				demod->state = DVBS2X_DEMOD_SEARCH;
				demod->frame_count = 0;
				dvbs2x_afc_init(&demod->afc, 0.95);
			}
		}
	}

	/*
	 * Report consumed samples.  Use the known frame length
	 * (header + data field) converted to sample rate.  Add a
	 * small margin for the RRC filter group delay on the first
	 * frame.
	 */
	if (demod->expected_frame_len > 0)
		*consumed = demod->expected_frame_len * sps;
	else
		*consumed = (DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_WH_LEN +
			     16384) * sps;

	if (*consumed > in_len)
		*consumed = in_len;

	return ret;
}
