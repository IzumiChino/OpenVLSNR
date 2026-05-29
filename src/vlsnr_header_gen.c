// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Walsh-Hadamard Header
 *
 * The VL-SNR header uses a 896-symbol Walsh-Hadamard sequence
 * for robust synchronization and MODCOD signaling at very low SNR.
 *
 * Walsh-Hadamard matrix of order 1024 is generated recursively:
 *   H(1) = [1]
 *   H(2n) = [H(n)  H(n)]
 *            [H(n) -H(n)]
 *
 * The first 896 elements of a selected row are used.
 * The row index encodes the MODCOD and format information.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2.5
 */

#include "vlsnr_header.h"
#include "dvbs2x_modcod.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dvbs2x_wh_generate(unsigned int index, int8_t *seq)
{
	unsigned int n;
	unsigned int order = DVBS2X_WH_ORDER;

	/*
	 * Walsh-Hadamard sequence generation using the bit-reversal
	 * property: H[row][col] = (-1)^(popcount(row & col))
	 *
	 * We only need the first 896 columns.
	 */
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		/* Count number of 1-bits in (index AND n) */
		unsigned int val = index & n;
		unsigned int count = 0;

		while (val) {
			count += val & 1;
			val >>= 1;
		}

		/* H[index][n] = (-1)^count */
		seq[n] = (count & 1) ? -1 : 1;
	}

	(void)order;
}

/*
 * Map MODCOD index to Walsh-Hadamard row index.
 * The standard defines specific WH row assignments for each MODCOD.
 *
 * These are placeholder values - the actual mapping is defined in
 * ETSI EN 302 307-2 Table 17b.
 */
static unsigned int modcod_to_wh_index(const struct dvbs2x_modcod *modcod)
{
	/*
	 * VL-SNR Set 1 (PLS 129): indices 0-5 map to WH rows
	 * VL-SNR Set 2 (PLS 131): indices 6-8 map to WH rows
	 *
	 * The format index within each set determines the WH row.
	 * Row selection ensures maximum Hamming distance between
	 * different MODCODs for robust detection.
	 */
	static const unsigned int wh_rows[DVBS2X_VLSNR_NUM_MODCODS] = {
		0, 1, 2, 3, 4, 5,	/* Set 1: MODCOD 1-6 */
		0, 1, 2,		/* Set 2: MODCOD 7-9 */
	};

	if (modcod->index < 1 || modcod->index > DVBS2X_VLSNR_NUM_MODCODS)
		return 0;

	return wh_rows[modcod->index - 1];
}

void dvbs2x_vlsnr_header_generate(const struct dvbs2x_modcod *modcod,
				  struct dvbs2x_complex *symbols)
{
	int8_t wh_seq[DVBS2X_VLSNR_WH_LEN];
	unsigned int wh_index;
	unsigned int n;
	double val, angle;

	/* Generate Walsh-Hadamard sequence for this MODCOD */
	wh_index = modcod_to_wh_index(modcod);
	dvbs2x_wh_generate(wh_index, wh_seq);

	/* First 2 symbols are zero padding */
	symbols[0].i = 0.0;
	symbols[0].q = 0.0;
	symbols[1].i = 0.0;
	symbols[1].q = 0.0;

	/* Map WH sequence to pi/2-BPSK symbols */
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
		val = (double)wh_seq[n];	/* +1 or -1 */
		angle = (double)((n + DVBS2X_VLSNR_PAD_LEN) % 4) * M_PI / 2.0;
		symbols[n + DVBS2X_VLSNR_PAD_LEN].i = val * cos(angle);
		symbols[n + DVBS2X_VLSNR_PAD_LEN].q = val * sin(angle);
	}
}

double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx)
{
	int8_t wh_seq[DVBS2X_VLSNR_WH_LEN];
	struct dvbs2x_complex ref[DVBS2X_VLSNR_WH_LEN];
	double best_corr = 0.0;
	unsigned int best_offset = 0;
	unsigned int best_modcod = 1;
	unsigned int mc, pos;
	unsigned int n;

	if (seg_len == 0)
		seg_len = DVBS2X_VLSNR_WH_LEN;

	/* Try each MODCOD's WH sequence */
	for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
		const struct dvbs2x_modcod *modcod;
		unsigned int wh_idx;
		double angle;

		modcod = dvbs2x_vlsnr_get_modcod(mc + 1);
		if (!modcod)
			continue;

		wh_idx = modcod_to_wh_index(modcod);
		dvbs2x_wh_generate(wh_idx, wh_seq);

		/* Generate reference symbols */
		for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
			double val = (double)wh_seq[n];

			angle = (double)(n % 4) * M_PI / 2.0;
			ref[n].i = val * cos(angle);
			ref[n].q = val * sin(angle);
		}

		/* Slide correlation window */
		for (pos = 0; pos + DVBS2X_VLSNR_WH_LEN <= len; pos++) {
			double corr_mag;
			unsigned int seg_start;
			double seg_corr_mag = 0.0;

			/*
			 * Segment-coherent correlation:
			 * Divide into segments, compute magnitude per
			 * segment, then sum magnitudes.
			 */
			for (seg_start = 0;
			     seg_start < DVBS2X_VLSNR_WH_LEN;
			     seg_start += seg_len) {
				unsigned int seg_end;
				double si = 0.0, sq = 0.0;

				seg_end = seg_start + seg_len;
				if (seg_end > DVBS2X_VLSNR_WH_LEN)
					seg_end = DVBS2X_VLSNR_WH_LEN;

				for (n = seg_start; n < seg_end; n++) {
					unsigned int idx = pos + n;

					si += symbols[idx].i * ref[n].i +
					      symbols[idx].q * ref[n].q;
					sq += symbols[idx].q * ref[n].i -
					      symbols[idx].i * ref[n].q;
				}

				seg_corr_mag += sqrt(si * si + sq * sq);
			}

			corr_mag = seg_corr_mag / DVBS2X_VLSNR_WH_LEN;

			if (corr_mag > best_corr) {
				best_corr = corr_mag;
				best_offset = pos;
				best_modcod = mc + 1;
			}
		}
	}

	if (offset)
		*offset = best_offset;
	if (modcod_idx)
		*modcod_idx = best_modcod;

	return best_corr;
}
