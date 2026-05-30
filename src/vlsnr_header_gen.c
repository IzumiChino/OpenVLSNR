// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Walsh-Hadamard Header
 *
 * The VL-SNR header is a 896-symbol pi/2-BPSK preamble used for frame
 * synchronization, MODCOD signalling and data-aided carrier recovery
 * at very low SNR.
 *
 * Each MODCOD's header is a PN-covered Walsh-Hadamard sequence:
 *
 *     c[n] = pn[n] * wh_row[n]            (+/-1)
 *     s[n] = c[n] * exp(j * n * pi/2)     (pi/2-BPSK)
 *
 * The common PN sequence (a maximal-length m-sequence) gives the
 * header a sharp aperiodic autocorrelation, so the frame position is
 * unambiguous.  The per-MODCOD Walsh-Hadamard cover is mutually
 * orthogonal over the 896-symbol window, so the MODCOD is identified
 * without ambiguity.  The combined sequence keeps a thumbtack
 * autocorrelation (sidelobes < 0.1) and inter-MODCOD correlation
 * (< 0.15), which is what makes acquisition possible far below 0 dB.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2.5 (sequence design here is
 * implementation-specific but follows the standard's intent).
 */

#include "vlsnr_header.h"
#include "dvbs2x_modcod.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Galois LFSR feedback mask for the degree-10 PN cover (period 1023) */
#define VLSNR_PN_MASK	0x44b

/*
 * Walsh-Hadamard rows assigned to MODCODs 1..9.  Chosen so that the
 * truncated (896-symbol) cross-correlation between any two rows is
 * exactly zero, giving unambiguous MODCOD identification.
 */
static const unsigned int vlsnr_wh_rows[DVBS2X_VLSNR_NUM_MODCODS] = {
	33, 61, 69, 131, 254, 292, 392, 411, 434,
};

void dvbs2x_wh_generate(unsigned int index, int8_t *seq)
{
	unsigned int n;

	/*
	 * Walsh-Hadamard sequence: H[index][n] = (-1)^popcount(index & n).
	 */
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		unsigned int val = index & n;
		unsigned int count = 0;

		while (val) {
			count += val & 1;
			val >>= 1;
		}
		seq[n] = (count & 1) ? -1 : 1;
	}
}

/* Generate the common PN cover (m-sequence, +/-1). */
static void vlsnr_pn_generate(int8_t *pn)
{
	unsigned int s = 1;
	unsigned int n;

	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		unsigned int lsb = s & 1;

		pn[n] = lsb ? -1 : 1;
		s >>= 1;
		if (lsb)
			s ^= VLSNR_PN_MASK;
	}
}

/*
 * Build the +/-1 header chip sequence c[n] = pn[n] * wh_row[n] for a
 * given Walsh-Hadamard row.
 */
static void vlsnr_chip_sequence(unsigned int wh_row, int8_t *chips)
{
	int8_t wh[DVBS2X_VLSNR_WH_LEN];
	int8_t pn[DVBS2X_VLSNR_WH_LEN];
	unsigned int n;

	dvbs2x_wh_generate(wh_row, wh);
	vlsnr_pn_generate(pn);
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++)
		chips[n] = (int8_t)(pn[n] * wh[n]);
}

static unsigned int modcod_to_wh_row(const struct dvbs2x_modcod *modcod)
{
	if (modcod->index < 1 || modcod->index > DVBS2X_VLSNR_NUM_MODCODS)
		return vlsnr_wh_rows[0];
	return vlsnr_wh_rows[modcod->index - 1];
}

/* Map a +/-1 chip sequence to pi/2-BPSK reference symbols. */
static void chips_to_symbols(const int8_t *chips,
			     struct dvbs2x_complex *symbols)
{
	unsigned int n;

	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		double val = (double)chips[n];
		double angle = (double)(n % 4) * M_PI / 2.0;

		symbols[n].i = val * cos(angle);
		symbols[n].q = val * sin(angle);
	}
}

void dvbs2x_vlsnr_header_generate(const struct dvbs2x_modcod *modcod,
				  struct dvbs2x_complex *symbols)
{
	int8_t chips[DVBS2X_VLSNR_WH_LEN];

	vlsnr_chip_sequence(modcod_to_wh_row(modcod), chips);
	chips_to_symbols(chips, symbols);
}

double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx)
{
	static struct dvbs2x_complex ref[DVBS2X_VLSNR_NUM_MODCODS]
					[DVBS2X_VLSNR_WH_LEN];
	static int ref_init;
	double best_corr = -1.0;
	unsigned int best_offset = 0;
	unsigned int best_modcod = 1;
	unsigned int mc, pos, n;

	if (seg_len == 0 || seg_len > DVBS2X_VLSNR_WH_LEN)
		seg_len = DVBS2X_VLSNR_WH_LEN;

	/* Pre-compute reference symbols once (lazy init) */
	if (!ref_init) {
		for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
			const struct dvbs2x_modcod *modcod;
			int8_t chips[DVBS2X_VLSNR_WH_LEN];

			modcod = dvbs2x_vlsnr_get_modcod(mc + 1);
			if (!modcod) {
				memset(ref[mc], 0, sizeof(ref[mc]));
				continue;
			}
			vlsnr_chip_sequence(modcod_to_wh_row(modcod), chips);
			chips_to_symbols(chips, ref[mc]);
		}
		ref_init = 1;
	}

	if (len < DVBS2X_VLSNR_WH_LEN)
		goto done;

	/* Slide a correlation window over the input */
	for (pos = 0; pos + DVBS2X_VLSNR_WH_LEN <= len; pos++) {
		for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
			double seg_mag = 0.0;
			unsigned int seg_start;
			double corr;

			/*
			 * Segment-coherent correlation: coherent sum within
			 * each segment, magnitudes summed across segments to
			 * tolerate residual frequency offset.
			 */
			for (seg_start = 0;
			     seg_start < DVBS2X_VLSNR_WH_LEN;
			     seg_start += seg_len) {
				unsigned int seg_end = seg_start + seg_len;
				double si = 0.0, sq = 0.0;

				if (seg_end > DVBS2X_VLSNR_WH_LEN)
					seg_end = DVBS2X_VLSNR_WH_LEN;

				for (n = seg_start; n < seg_end; n++) {
					const struct dvbs2x_complex *r =
						&symbols[pos + n];
					const struct dvbs2x_complex *rf =
						&ref[mc][n];

					si += r->i * rf->i + r->q * rf->q;
					sq += r->q * rf->i - r->i * rf->q;
				}
				seg_mag += sqrt(si * si + sq * sq);
			}

			corr = seg_mag / (double)DVBS2X_VLSNR_WH_LEN;
			if (corr > best_corr) {
				best_corr = corr;
				best_offset = pos;
				best_modcod = mc + 1;
			}
		}
	}

done:
	if (offset)
		*offset = best_offset;
	if (modcod_idx)
		*modcod_idx = best_modcod;
	return best_corr;
}
