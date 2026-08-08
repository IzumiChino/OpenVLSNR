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
#include <limits.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define M_SQRT1_2_D	0.70710678118654752440
#define DVBS2X_GOLD_CODE_MAX	262141

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

/* Limit oscillator drift without paying for transcendental calls per symbol. */
#define DEROTATE_REANCHOR	256

/*
 * Frame-acquisition search window (symbols).  The demodulator expects
 * the frame to start within this many symbols of the buffer start, as
 * a receiver would after coarse burst detection.  Bounding the search
 * keeps acquisition cost independent of the buffer length.
 */
#define ACQ_WINDOW	4096

/*
 * Number of payload symbols (after spreading) and the resulting
 * data-field length including pilot blocks, from the VL-SNR layout.
 * Returns 0 on success.
 */
static int frame_geometry(const struct dvbs2x_modcod *mc,
			  unsigned int *num_tx_sym,
			  unsigned int *data_field_len)
{
	struct dvbs2x_vlsnr_layout lay;

	if (dvbs2x_vlsnr_build_layout(mc, &lay) < 0)
		return -1;
	*num_tx_sym = lay.num_data;
	*data_field_len = lay.field_len;
	dvbs2x_vlsnr_free_layout(&lay);
	return 0;
}

/* Rotate symbols [base, base+span) by exp(-j(2*pi*f*n + phi)), n from 0. */
static void derotate(struct dvbs2x_complex *sym, unsigned int base,
		     unsigned int span, double f, double phi)
{
	double step = 2.0 * M_PI * f;
	double step_c = cos(step), step_s = sin(step);
	double c = cos(phi), s = sin(phi);
	unsigned int n;

	for (n = 0; n < span; n++) {
		struct dvbs2x_complex *r = &sym[base + n];
		double ti = r->i * c + r->q * s;
		double tq = r->q * c - r->i * s;
		double next_c, next_s;

		r->i = ti;
		r->q = tq;

		next_c = c * step_c - s * step_s;
		next_s = s * step_c + c * step_s;
		c = next_c;
		s = next_s;

		/* Bound recurrence error on long normal frames. */
		if ((n + 1) % DEROTATE_REANCHOR == 0) {
			double angle = step * (double)(n + 1) + phi;

			c = cos(angle);
			s = sin(angle);
		}
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
 * every pilot block (from the data-field layout), unwrap, weighted-
 * linear-fit phase against frame position, and de-rotate the whole data
 * field.  The data field must already be descrambled, so the pilots sit
 * at their nominal reference (1+j)/sqrt(2).
 */
static void residual_carrier_track(struct dvbs2x_complex *sym,
				   unsigned int wh_start,
				   const struct dvbs2x_vlsnr_layout *lay,
				   const struct dvbs2x_complex *wh_ref)
{
	unsigned int data_start = wh_start + DVBS2X_VLSNR_WH_LEN + 2;
	unsigned int n, i;
	double ci, cq;
	double sw, swp, swt, swpp, swpt, denom, a, b;
	double prev_ph;
	int have_prev = 0;

	sw = swp = swt = swpp = swpt = 0.0;

	/* Header phase sample (anchor) at the header centre */
	ci = 0.0;
	cq = 0.0;
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		const struct dvbs2x_complex *r = &sym[wh_start + n];

		ci += r->i * wh_ref[n].i + r->q * wh_ref[n].q;
		cq += r->q * wh_ref[n].i - r->i * wh_ref[n].q;
	}
	{
		double pos = (double)(DVBS2X_VLSNR_WH_LEN - 1) / 2.0;
		double phv = atan2(cq, ci);
		double wv = (double)DVBS2X_VLSNR_WH_LEN;

		sw += wv;
		swp += wv * pos;
		swt += wv * phv;
		swpp += wv * pos * pos;
		swpt += wv * pos * phv;
		prev_ph = phv;
		have_prev = 1;
	}

	/* One phase sample per pilot block (vs reference (1+j)/sqrt2) */
	i = 0;
	while (i < lay->field_len) {
		unsigned int blk;

		if (!lay->is_pilot[i]) {
			i++;
			continue;
		}
		blk = 0;
		ci = 0.0;
		cq = 0.0;
		while (i + blk < lay->field_len && lay->is_pilot[i + blk]) {
			const struct dvbs2x_complex *r =
				&sym[data_start + i + blk];

			ci += (r->i + r->q) * M_SQRT1_2_D;
			cq += (r->q - r->i) * M_SQRT1_2_D;
			blk++;
		}
		{
			double pos = (double)DVBS2X_VLSNR_WH_LEN + (double)i +
				     (double)(blk - 1) / 2.0;
			double phv = atan2(cq, ci);
			double wv = (double)blk;

			if (have_prev) {
				double d = phv - prev_ph;

				while (d > M_PI) {
					phv -= 2.0 * M_PI;
					d = phv - prev_ph;
				}
				while (d < -M_PI) {
					phv += 2.0 * M_PI;
					d = phv - prev_ph;
				}
			}
			prev_ph = phv;
			have_prev = 1;

			sw += wv;
			swp += wv * pos;
			swt += wv * phv;
			swpp += wv * pos * pos;
			swpt += wv * pos * phv;
		}
		i += blk;
	}

	/* Weighted linear fit phase = a*pos + b */
	denom = sw * swpp - swp * swp;
	if (fabs(denom) < 1e-9) {
		a = 0.0;
		b = swt / sw;
	} else {
		a = (sw * swpt - swp * swt) / denom;
		b = (swt - a * swp) / sw;
	}

	/* De-rotate the data field by the fitted phase line. */
	derotate(sym, data_start, lay->field_len, a / (2.0 * M_PI),
		 a * (double)DVBS2X_VLSNR_WH_LEN + b);
}

int dvbs2x_demodulator_init(struct dvbs2x_demodulator *demod,
			    double rolloff,
			    unsigned int sps,
			    unsigned int pl_scrambling_idx)
{
	if (!demod)
		return DVBS2X_ERR_PARAM;
	memset(demod, 0, sizeof(*demod));
	demod->modcod = NULL;	/* determined after frame sync */

	if (sps < 2 || rolloff < 0.05 || rolloff > 0.35 ||
	    pl_scrambling_idx > DVBS2X_GOLD_CODE_MAX)
		return DVBS2X_ERR_PARAM;

	if (dvbs2x_rrc_filter_init(&demod->rx_filter, rolloff, sps, 16) < 0)
		return DVBS2X_ERR_PARAM;

	dvbs2x_timing_sync_init(&demod->timing, sps, 1e-3);
	dvbs2x_freq_coarse_init(&demod->freq_coarse, 1e-4);
	dvbs2x_freq_fine_init(&demod->freq_fine, 4);
	dvbs2x_phase_est_init(&demod->phase, 0.3, 1);
	dvbs2x_afc_init(&demod->afc, 0.95);
	dvbs2x_scrambler_init(&demod->descrambler, pl_scrambling_idx);

	demod->consecutive_successes = 0;
	demod->consecutive_failures = 0;

	return 0;
}

void dvbs2x_demodulator_destroy(struct dvbs2x_demodulator *demod)
{
	if (!demod)
		return;
	dvbs2x_ldpc_decoder_free(&demod->ldpc_dec);
	free(demod->stream_buf);
	demod->stream_buf = NULL;
	demod->stream_len = 0;
	demod->stream_cap = 0;
}

int dvbs2x_demodulate_bbframe_symbols_ex(struct dvbs2x_demodulator *demod,
					 const struct dvbs2x_complex *input,
					 unsigned int in_len,
					 double noise_var,
					 uint8_t *bbframe,
					 unsigned int frame_capacity,
					 unsigned int *frame_len)
{
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *work = NULL;
	struct dvbs2x_complex *wh_ref = NULL;
	struct dvbs2x_complex *data_sym = NULL;
	struct dvbs2x_complex *pilots = NULL;
	double *sym_llr = NULL;
	double *llr = NULL;
	double *deint = NULL;
	uint8_t *ldpc_out = NULL;
	uint8_t *bch_cw = NULL;
	struct dvbs2x_vlsnr_layout lay = { 0, 0, 0, NULL };
	unsigned int wh_start = 0, modcod_idx = 0;
	unsigned int search_len;
	unsigned int data_field_len, data_start;
	unsigned int ndata, tx_coded;
	unsigned int iter_used = 0;
	double conf, demap_nv, nv_est, esn0_db;
	int ret = DVBS2X_ERR_FEC;

	if (!frame_len)
		return DVBS2X_ERR_PARAM;
	*frame_len = 0;
	if (!demod || !input || !bbframe || noise_var < 0.0)
		return DVBS2X_ERR_PARAM;
	memset(&demod->last_stats, 0, sizeof(demod->last_stats));
	demod->last_stats.result = DVBS2X_ERR_NOSYNC;
	if (demod->cancel_requested) {
		demod->last_stats.result = DVBS2X_ERR_CANCELLED;
		return DVBS2X_ERR_CANCELLED;
	}

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
	demod->last_stats.sync_confidence = conf;
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
	demod->last_stats.modcod = modcod_idx;

	if (dvbs2x_vlsnr_build_layout(mc, &lay) < 0) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	data_field_len = lay.field_len;
	data_start = wh_start + DVBS2X_VLSNR_WH_LEN + 2;
#ifdef DEBUG
	fprintf(stderr,
		"[dbg] wh_start=%u modcod=%u conf=%.3f data=%u "
		"dfl=%u in_len=%u\n",
		wh_start, modcod_idx, conf, lay.num_data,
		data_field_len, in_len);
#endif
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
	demod->last_stats.esn0_db = esn0_db;
#ifdef DEBUG
	fprintf(stderr, "[dbg] nv_est=%.4f esn0_est=%.2f dB\n",
		nv_est, esn0_db);
#endif

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

	/*
	 * Descramble the data field (payload + pilots).  The VL-SNR data
	 * field reads the PL scrambler from index VLSNR_HDR_LEN (900);
	 * non-QPSK payload is descrambled by real +/-1, pilots by 4-phase.
	 */
	dvbs2x_scrambler_reset(&demod->descrambler);
	dvbs2x_scrambler_seek(&demod->descrambler, DVBS2X_VLSNR_HDR_LEN);
	dvbs2x_descramble_field(&demod->descrambler, work + data_start,
				lay.field_len, lay.is_pilot,
				mc->modulation == DVBS2X_MOD_QPSK);

	/* Residual phase/frequency tracking from header + pilots */
	residual_carrier_track(work, wh_start, &lay, wh_ref + 2);

	/* Extract payload and pilot symbols per the layout */
	data_sym = malloc(lay.num_data * sizeof(struct dvbs2x_complex));
	pilots = malloc((lay.num_pilot + 1) * sizeof(struct dvbs2x_complex));
	if (!data_sym || !pilots) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	ndata = dvbs2x_pilot_extract(work + data_start, &lay,
				     data_sym, pilots);

	/*
	 * Demapper noise variance: estimate from pilot scatter around
	 * (1+j)/sqrt(2), or use the caller-supplied value if given.
	 */
	if (noise_var > 0.0) {
		demap_nv = noise_var;
	} else if (lay.num_pilot) {
		double acc = 0.0;
		unsigned int pi;

		for (pi = 0; pi < lay.num_pilot; pi++) {
			double di = pilots[pi].i - M_SQRT1_2_D;
			double dq = pilots[pi].q - M_SQRT1_2_D;

			acc += di * di + dq * dq;
		}
		demap_nv = acc / (2.0 * (double)lay.num_pilot);
		if (demap_nv < 1e-4)
			demap_nv = 1e-4;
	} else {
		demap_nv = nv_est > 1e-4 ? nv_est : 1e-4;
	}
#ifdef DEBUG
	fprintf(stderr, "[dbg] demap_nv=%.5f esn0~%.2f dB\n",
		demap_nv, 10.0 * log10(1.0 / (2.0 * demap_nv + 1e-12)));
#endif

	tx_coded = dvbs2x_tx_coded_bits(mc);

	/* Per-symbol soft demapping (period-2 pi/2-BPSK or QPSK) */
	sym_llr = malloc((2 * ndata + 2) * sizeof(double));
	llr = malloc(tx_coded * sizeof(double));
	deint = malloc(mc->fec_len * sizeof(double));
	if (!sym_llr || !llr || !deint) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}

	if (mc->modulation == DVBS2X_MOD_QPSK)
		dvbs2x_demod_qpsk(data_sym, sym_llr, ndata, demap_nv);
	else
		dvbs2x_demod_pi2bpsk(data_sym, sym_llr, ndata, demap_nv);

	/*
	 * SF2 despread: each coded bit was carried by two symbols at the
	 * even and odd pi/2-BPSK phases, so sum their LLRs.  Otherwise the
	 * per-symbol LLRs are already the coded-bit LLRs.
	 */
	if (mc->has_spread)
		dvbs2x_demod_despread_llr(sym_llr, llr, ndata / 2);
	else if (mc->modulation == DVBS2X_MOD_QPSK)
		memcpy(llr, sym_llr, 2 * ndata * sizeof(double));
	else
		memcpy(llr, sym_llr, ndata * sizeof(double));

	/* Deinterleave (operates on tx_coded bits) */
	dvbs2x_deinterleave(mc, llr, deint);

	/*
	 * Depuncture and deshorten: expand tx_coded LLRs back to the full
	 * fec_len LDPC codeword.
	 *   deint[0 .. k_ldpc-xs-1]        = info LLRs
	 *   deint[k_ldpc-xs .. tx_coded-1] = unpunctured parity
	 *   full_llr[0..xs-1]              = +clamp (shortened, known zero)
	 *   full_llr[xs..k_ldpc-1]         = info LLRs
	 *   full_llr[k_ldpc..fec_len-1]    = parity, 0.0 at punctured pos
	 */
	{
		double *full_llr;
		unsigned int i, p_idx, d_idx;

		full_llr = malloc(mc->fec_len * sizeof(double));
		if (!full_llr) {
			ret = DVBS2X_ERR_NOMEM;
			goto out;
		}

		for (i = 0; i < mc->xs; i++)
			full_llr[i] = 40.0;
		for (i = mc->xs; i < mc->k_ldpc; i++)
			full_llr[i] = deint[i - mc->xs];

		d_idx = mc->k_ldpc - mc->xs;
		p_idx = 0;
		for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
			if (mc->xp > 0 && p_idx < mc->xp &&
			    i == p_idx * mc->p_period) {
				full_llr[mc->k_ldpc + i] = 0.0;
				p_idx++;
			} else {
				full_llr[mc->k_ldpc + i] = deint[d_idx++];
			}
		}

		free(deint);
		deint = full_llr;
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
	demod->ldpc_dec.cancel_flag = &demod->cancel_requested;
	ldpc_out = malloc(mc->k_ldpc);
	if (!ldpc_out) {
		ret = DVBS2X_ERR_NOMEM;
		goto out;
	}
	if (dvbs2x_ldpc_decode(&demod->ldpc_dec, deint, ldpc_out,
			       &iter_used) < 0)
		ret = demod->cancel_requested ? DVBS2X_ERR_CANCELLED :
			DVBS2X_ERR_FEC;
	demod->last_stats.ldpc_iterations = iter_used;
	if (ret == DVBS2X_ERR_CANCELLED)
		goto out;
		/* Continue to BCH - best-effort decode */
#ifdef DEBUG
	fprintf(stderr, "[dbg] ldpc iter=%u/%u\n", iter_used,
		DVBS2X_LDPC_MAX_ITER);
#endif

	/*
	 * BCH decode: the LDPC information word is xs shortening zeros
	 * followed by the n_bch-bit BCH codeword; skip the zeros.
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
	memcpy(bch_cw, ldpc_out + mc->xs, mc->n_bch);
	{
		int bret = dvbs2x_bch_decode(&demod->bch_dec, bch_cw);
#ifdef DEBUG
		fprintf(stderr, "[dbg] bch ret=%d\n", bret);
#endif
		if (bret < 0) {
			ret = DVBS2X_ERR_CRC;
			goto out;
		}
	}

	*frame_len = mc->k_bch;
	if (frame_capacity < *frame_len) {
		ret = DVBS2X_ERR_SHORT;
		goto out;
	}
	memcpy(bbframe, bch_cw, *frame_len);
	ret = 0;

out:
	free(work);
	free(wh_ref);
	free(data_sym);
	free(pilots);
	free(sym_llr);
	free(llr);
	free(deint);
	free(ldpc_out);
	free(bch_cw);
	dvbs2x_vlsnr_free_layout(&lay);
	if (demod)
		demod->last_stats.result = ret;
	return ret;
}

int dvbs2x_demodulator_get_stats(const struct dvbs2x_demodulator *demod,
				 struct dvbs2x_demod_stats *stats)
{
	if (!demod || !stats)
		return DVBS2X_ERR_PARAM;
	*stats = demod->last_stats;
	return 0;
}

void dvbs2x_demodulator_request_cancel(struct dvbs2x_demodulator *demod)
{
	if (demod)
		demod->cancel_requested = 1;
}

int dvbs2x_demodulate_symbols_ex(struct dvbs2x_demodulator *demod,
				 const struct dvbs2x_complex *input,
				 unsigned int in_len,
				 double noise_var,
				 uint8_t *user_data,
				 unsigned int user_capacity,
				 unsigned int *user_len)
{
	uint8_t *bbframe;
	unsigned int frame_len = 0;
	int ret;

	if (!user_len)
		return DVBS2X_ERR_PARAM;
	*user_len = 0;
	if (!demod || !input || !user_data || noise_var < 0.0)
		return DVBS2X_ERR_PARAM;
	bbframe = malloc(DVBS2X_LDPC_NORMAL);
	if (!bbframe)
		return DVBS2X_ERR_NOMEM;
	ret = dvbs2x_demodulate_bbframe_symbols_ex(demod, input, in_len,
						   noise_var, bbframe,
						   DVBS2X_LDPC_NORMAL,
						   &frame_len);
	if (ret < 0)
		goto out;
	if (!demod->modcod || frame_len != demod->modcod->k_bch) {
		ret = DVBS2X_ERR_PARAM;
		goto out;
	}
	dvbs2x_bb_frame_init(&demod->bb_ctx, demod->modcod,
				 DVBS2X_STREAM_GS);
	ret = dvbs2x_bb_frame_parse_ex(&demod->bb_ctx, bbframe,
				       user_data, user_capacity, user_len);
	if (ret < 0 && ret != DVBS2X_ERR_SHORT)
		ret = DVBS2X_ERR_CRC;
out:
	free(bbframe);
	return ret;
}

int dvbs2x_demodulate_symbols(struct dvbs2x_demodulator *demod,
			      const struct dvbs2x_complex *input,
			      unsigned int in_len,
			      double noise_var,
			      uint8_t *user_data,
			      unsigned int *user_len)
{
	return dvbs2x_demodulate_symbols_ex(demod, input, in_len, noise_var,
					    user_data, UINT_MAX, user_len);
}

int dvbs2x_demodulate_ex(struct dvbs2x_demodulator *demod,
			 const struct dvbs2x_complex *input,
			 unsigned int in_len,
			 uint8_t *user_data,
			 unsigned int user_capacity,
			 unsigned int *user_len)
{
	struct dvbs2x_complex *filtered = NULL;
	struct dvbs2x_complex *symbols = NULL;
	unsigned int filt_len = 0, sym_len = 0;
	int ret = DVBS2X_ERR_NOMEM;

	if (!user_len)
		return DVBS2X_ERR_PARAM;
	*user_len = 0;
	if (!demod || !input || !user_data)
		return DVBS2X_ERR_PARAM;

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

	ret = dvbs2x_demodulate_symbols_ex(demod, symbols, sym_len, 0.0,
					   user_data, user_capacity, user_len);

out:
	free(filtered);
	free(symbols);
	return ret;
}

int dvbs2x_demodulate_bbframe_ex(struct dvbs2x_demodulator *demod,
				 const struct dvbs2x_complex *input,
				 unsigned int in_len,
				 uint8_t *bbframe,
				 unsigned int frame_capacity,
				 unsigned int *frame_len)
{
	struct dvbs2x_complex *filtered = NULL;
	struct dvbs2x_complex *symbols = NULL;
	unsigned int filt_len = 0, sym_len = 0;
	int ret = DVBS2X_ERR_NOMEM;

	if (!frame_len)
		return DVBS2X_ERR_PARAM;
	*frame_len = 0;
	if (!demod || !input || !bbframe)
		return DVBS2X_ERR_PARAM;
	filtered = malloc(in_len * sizeof(*filtered));
	if (!filtered)
		goto out;
	dvbs2x_rrc_filter_reset(&demod->rx_filter);
	dvbs2x_rrc_filter_apply(&demod->rx_filter, input, in_len,
				filtered, &filt_len);
	symbols = malloc((filt_len / demod->timing.sps + 2) *
			 sizeof(*symbols));
	if (!symbols)
		goto out;
	dvbs2x_timing_sync_init(&demod->timing, demod->timing.sps, 1e-3);
	dvbs2x_timing_sync_process(&demod->timing, filtered, filt_len,
				   symbols, &sym_len);
	ret = dvbs2x_demodulate_bbframe_symbols_ex(demod, symbols, sym_len,
						  0.0, bbframe,
						  frame_capacity, frame_len);
out:
	free(filtered);
	free(symbols);
	return ret;
}

int dvbs2x_demodulate(struct dvbs2x_demodulator *demod,
		      const struct dvbs2x_complex *input,
		      unsigned int in_len,
		      uint8_t *user_data,
		      unsigned int *user_len)
{
	return dvbs2x_demodulate_ex(demod, input, in_len, user_data,
				    UINT_MAX, user_len);
}

/*
 * Lock management thresholds: consecutive successes to declare TRACK,
 * consecutive failures to revert to SEARCH.
 */
#define LOCK_THRESH	3
#define LOSS_THRESH	3

void dvbs2x_demod_lock_update(struct dvbs2x_demodulator *demod, int success)
{
	if (success) {
		demod->consecutive_successes++;
		demod->consecutive_failures = 0;
		if (demod->state == DVBS2X_DEMOD_SEARCH)
			demod->state = DVBS2X_DEMOD_ACQUIRE;
		if (demod->state == DVBS2X_DEMOD_ACQUIRE &&
		    demod->consecutive_successes >= LOCK_THRESH)
			demod->state = DVBS2X_DEMOD_TRACK;
		return;
	}

	demod->consecutive_successes = 0;
	demod->consecutive_failures++;
	if (demod->state != DVBS2X_DEMOD_TRACK ||
	    demod->consecutive_failures < LOSS_THRESH)
		return;

	demod->state = DVBS2X_DEMOD_SEARCH;
	demod->consecutive_failures = 0;
	demod->expected_frame_len = 0;
	dvbs2x_afc_init(&demod->afc, 0.95);
}

static int demodulate_stream_ex(struct dvbs2x_demodulator *demod,
				const struct dvbs2x_complex *input,
				unsigned int in_len, uint8_t *output,
				unsigned int output_capacity,
				unsigned int *output_len,
				unsigned int *consumed, int return_bbframe)
{
	struct dvbs2x_complex *new_buf;
	struct dvbs2x_complex *decode_buf = NULL;
	unsigned int min_samples;
	unsigned int decode_len;
	unsigned int frame_samples = 0;
	unsigned int new_cap;
	int ret;

	if (!demod || !output_len || !consumed || !output ||
	    (in_len && !input))
		return DVBS2X_ERR_PARAM;
	*consumed = 0;
	*output_len = 0;

	if (in_len) {
		if (in_len > UINT_MAX - demod->stream_len)
			return DVBS2X_ERR_PARAM;
		if (demod->stream_len + in_len > demod->stream_cap) {
			new_cap = demod->stream_cap ? demod->stream_cap : 4096;
			while (new_cap < demod->stream_len + in_len) {
				if (new_cap > UINT_MAX / 2) {
					new_cap = demod->stream_len + in_len;
					break;
				}
				new_cap *= 2;
			}
			new_buf = realloc(demod->stream_buf,
					  new_cap * sizeof(*new_buf));
			if (!new_buf)
				return DVBS2X_ERR_NOMEM;
			demod->stream_buf = new_buf;
			demod->stream_cap = new_cap;
		}
		memcpy(demod->stream_buf + demod->stream_len, input,
		       in_len * sizeof(*input));
		demod->stream_len += in_len;
		*consumed = in_len;
	}

	/*
	 * A short frame is the smallest decodable unit.  Keep smaller
	 * chunks buffered without advancing any receiver state.
	 */
	min_samples = DVBS2X_VLSNR_FRAME_SHORT * demod->timing.sps;
	if (demod->stream_len < min_samples)
		return DVBS2X_ERR_SHORT;
	if (demod->expected_frame_len &&
	    demod->stream_len < demod->expected_frame_len * demod->timing.sps)
		return DVBS2X_ERR_SHORT;

	/*
	 * Append zero tail samples to make the final interpolation points
	 * available when the buffer ends exactly at a frame boundary.
	 */
	if (demod->stream_len > UINT_MAX - demod->rx_filter.num_taps)
		return DVBS2X_ERR_PARAM;
	decode_len = demod->stream_len + demod->rx_filter.num_taps;
	decode_buf = calloc(decode_len, sizeof(*decode_buf));
	if (!decode_buf)
		return DVBS2X_ERR_NOMEM;
	memcpy(decode_buf, demod->stream_buf,
	       demod->stream_len * sizeof(*decode_buf));
	if (return_bbframe)
		ret = dvbs2x_demodulate_bbframe_ex(demod, decode_buf,
						    decode_len, output,
						    output_capacity,
						    output_len);
	else
		ret = dvbs2x_demodulate_ex(demod, decode_buf, decode_len,
					  output, output_capacity, output_len);
	free(decode_buf);

	if (demod->modcod) {
		unsigned int tx_sym, field_len;

		if (frame_geometry(demod->modcod, &tx_sym, &field_len) == 0) {
			demod->expected_frame_len = DVBS2X_PLHEADER_LEN +
				DVBS2X_VLSNR_HDR_LEN + field_len;
			frame_samples = demod->expected_frame_len *
				demod->timing.sps;
		}
	}

	if (ret == DVBS2X_ERR_SHORT)
		return ret;

	/* Update state machine */
	dvbs2x_demod_lock_update(demod, ret == 0);

	/*
	 * Remove exactly one decoded or identified frame.  If no header was
	 * found, discard one sample so SEARCH can make forward progress while
	 * retaining the rest of the acquisition window.
	 */
	if (!frame_samples || frame_samples > demod->stream_len)
		frame_samples = 1;
	demod->stream_len -= frame_samples;
	memmove(demod->stream_buf, demod->stream_buf + frame_samples,
		demod->stream_len * sizeof(*demod->stream_buf));

	return ret;
}

int dvbs2x_demodulate_stream_ex(struct dvbs2x_demodulator *demod,
				const struct dvbs2x_complex *input,
				unsigned int in_len,
				uint8_t *user_data,
				unsigned int user_capacity,
				unsigned int *user_len,
				unsigned int *consumed)
{
	return demodulate_stream_ex(demod, input, in_len, user_data,
				    user_capacity, user_len, consumed, 0);
}

int dvbs2x_demodulate_bbframe_stream_ex(struct dvbs2x_demodulator *demod,
					const struct dvbs2x_complex *input,
					unsigned int in_len,
					uint8_t *bbframe,
					unsigned int frame_capacity,
					unsigned int *frame_len,
					unsigned int *consumed)
{
	return demodulate_stream_ex(demod, input, in_len, bbframe,
				    frame_capacity, frame_len, consumed, 1);
}

int dvbs2x_demodulate_stream(struct dvbs2x_demodulator *demod,
			     const struct dvbs2x_complex *input,
			     unsigned int in_len,
			     uint8_t *user_data,
			     unsigned int *user_len,
			     unsigned int *consumed)
{
	return dvbs2x_demodulate_stream_ex(demod, input, in_len, user_data,
					   UINT_MAX, user_len, consumed);
}
