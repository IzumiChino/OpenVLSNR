// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Walsh-Hadamard Header
 *
 * Implements the VL-SNR header as specified in ETSI EN 302 307-2
 * clause 5.5.2.5.  The header is 900 pi/2-BPSK symbols:
 *
 *     [2 zero-pad] [896-bit WH pattern] [2 zero-pad]
 *
 * The 896-bit pattern is constructed from a fixed 16x56 base matrix.
 * Each of the 16 rows is kept or inverted according to a 16-element
 * Walsh-Hadamard sign sequence (Table 18b), then the rows are
 * concatenated.  The sign sequence selects the MODCOD.
 *
 * Reference: ETSI EN 302 307-2 V1.3.1, clause 5.5.2.5, Table 18b
 */

#include "vlsnr_header.h"
#include "dvbs2x_modcod.h"
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Base matrix: 16 rows of 56 bits (7 bytes each), read left-to-right,
 * top-to-bottom.  From ETSI EN 302 307-2 clause 5.5.2.5.
 */
#define VLSNR_ROWS	16
#define VLSNR_COLS	56
#define VLSNR_ROW_BYTES	7

static const uint8_t vlsnr_base_matrix[VLSNR_ROWS][VLSNR_ROW_BYTES] = {
	{0xFB, 0xF2, 0x3E, 0x83, 0x7F, 0x9B, 0xC4},
	{0x98, 0x70, 0x8E, 0x0B, 0x39, 0x34, 0x5E},
	{0xF6, 0xA2, 0xC9, 0xFE, 0x1B, 0x17, 0x37},
	{0x84, 0x18, 0xD9, 0x5A, 0x6F, 0x99, 0x7A},
	{0x7B, 0x7D, 0x7B, 0x3E, 0x9F, 0xC9, 0xEA},
	{0x5E, 0x78, 0xBA, 0x03, 0xA6, 0xD5, 0x1A},
	{0x27, 0x9C, 0xC2, 0x65, 0x43, 0xEC, 0xD0},
	{0x34, 0x2B, 0x04, 0x98, 0xBF, 0x3D, 0x7D},
	{0xAD, 0xD0, 0x36, 0xE9, 0xD5, 0x31, 0x2F},
	{0x10, 0x61, 0xC6, 0xDF, 0x82, 0x62, 0x37},
	{0x72, 0xD3, 0xE0, 0x90, 0x73, 0x84, 0xC7},
	{0x3B, 0xD5, 0xAC, 0xEE, 0x25, 0xE2, 0xC9},
	{0x59, 0x08, 0x7D, 0x82, 0x61, 0x5A, 0xDA},
	{0xE9, 0xAF, 0x01, 0x72, 0xCF, 0x9D, 0xA7},
	{0x3F, 0x48, 0x35, 0xA4, 0x06, 0x3F, 0x07},
	{0x23, 0xC9, 0xAE, 0xEC, 0xF2, 0xED, 0x41},
};

/*
 * Walsh-Hadamard sign patterns (Table 18b).  +1 = keep row,
 * -1 = invert row.  Indexed by Annex-I index (0..15).
 */
static const int8_t vlsnr_wh_signs[16][VLSNR_ROWS] = {
	{+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1,+1},
	{+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1,+1,-1},
	{+1,+1,-1,-1,+1,+1,-1,-1,+1,+1,-1,-1,+1,+1,-1,-1},
	{+1,-1,-1,+1,+1,-1,-1,+1,+1,-1,-1,+1,+1,-1,-1,+1},
	{+1,+1,+1,+1,-1,-1,-1,-1,+1,+1,+1,+1,-1,-1,-1,-1},
	{+1,-1,-1,+1,-1,+1,+1,-1,+1,-1,-1,+1,-1,+1,+1,-1},
	{+1,+1,-1,-1,+1,+1,-1,-1,-1,-1,+1,+1,-1,-1,+1,+1},
	{+1,-1,-1,+1,+1,-1,-1,+1,-1,+1,+1,-1,-1,+1,+1,-1},
	{+1,+1,+1,+1,-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1},
	{+1,+1,-1,-1,-1,-1,+1,+1,+1,+1,-1,-1,-1,-1,+1,+1},
	{+1,-1,+1,-1,-1,+1,-1,+1,+1,-1,+1,-1,-1,+1,-1,+1},
	{-1,-1,-1,-1,-1,-1,-1,-1,+1,+1,+1,+1,+1,+1,+1,+1},
	{+1,-1,+1,-1,+1,-1,+1,-1,-1,+1,-1,+1,-1,+1,-1,+1},
	{+1,-1,+1,-1,-1,+1,-1,+1,-1,+1,-1,+1,+1,-1,+1,-1},
	{+1,+1,-1,-1,-1,-1,+1,+1,-1,-1,+1,+1,+1,+1,-1,-1},
	{+1,-1,-1,+1,-1,+1,+1,-1,-1,+1,+1,-1,+1,-1,-1,+1},
};

/*
 * Map MODCOD index (1-9) to Annex-I WH index (Table 18b).
 * Set 1 (MODCODs 1-6): indices 0-5
 * Set 2 (MODCODs 7-9): indices 9-11
 */
static const unsigned int modcod_to_annex_i[DVBS2X_VLSNR_NUM_MODCODS] = {
	0, 1, 2, 3, 4, 5, 9, 10, 11,
};

/*
 * Generate the 896-bit VL-SNR pattern for a given Annex-I index.
 * Output: 896 chips (+1/-1).
 */
static void vlsnr_generate_pattern(unsigned int annex_idx,
				   int8_t *chips)
{
	unsigned int row, col, bit;
	const int8_t *signs;

	if (annex_idx >= 16)
		annex_idx = 0;
	signs = vlsnr_wh_signs[annex_idx];

	for (row = 0; row < VLSNR_ROWS; row++) {
		for (col = 0; col < VLSNR_COLS; col++) {
			unsigned int byte_idx = col / 8;
			unsigned int bit_idx = 7 - (col % 8);

			bit = (vlsnr_base_matrix[row][byte_idx] >>
			       bit_idx) & 1;

			/* bit 0 -> +1, bit 1 -> -1 */
			chips[row * VLSNR_COLS + col] =
				(bit ? -1 : +1) * signs[row];
		}
	}
}

/*
 * Map a +/-1 chip sequence to period-2 pi/2-BPSK symbols, matching the
 * gr-dtv header mapping m_bpsk[i & 1][b] (b = 0 for chip +1, b = 1 for
 * chip -1):
 *   even index: chip +1 -> (+,+)/sqrt2, chip -1 -> (-,-)/sqrt2
 *   odd  index: chip +1 -> (-,+)/sqrt2, chip -1 -> (+,-)/sqrt2
 */
static void chips_to_symbols(const int8_t *chips, unsigned int len,
			     struct dvbs2x_complex *symbols,
			     unsigned int sym_offset)
{
	const double r = 0.70710678118654752440;
	unsigned int n;

	for (n = 0; n < len; n++) {
		int b = chips[n] < 0;	/* chip -1 -> bit 1 */
		unsigned int idx = sym_offset + n;

		if ((idx & 1) == 0) {
			symbols[n].i = b ? -r : r;
			symbols[n].q = b ? -r : r;
		} else {
			symbols[n].i = b ? r : -r;
			symbols[n].q = b ? -r : r;
		}
	}
}

void dvbs2x_vlsnr_header_generate(const struct dvbs2x_modcod *modcod,
				  struct dvbs2x_complex *symbols)
{
	int8_t chips[DVBS2X_VLSNR_HDR_LEN];
	unsigned int annex_idx;

	if (modcod->index < 1 || modcod->index > DVBS2X_VLSNR_NUM_MODCODS)
		return;
	annex_idx = modcod_to_annex_i[modcod->index - 1];

	/*
	 * Full 900-symbol header: 2 zero-pad bits, the 896-bit WH pattern,
	 * 2 zero-pad bits, all mapped period-2 (gr-dtv maps the entire
	 * header with m_bpsk[i & 1][b], pads included).
	 */
	chips[0] = +1;
	chips[1] = +1;
	vlsnr_generate_pattern(annex_idx, chips + 2);
	chips[2 + DVBS2X_VLSNR_WH_LEN] = +1;
	chips[3 + DVBS2X_VLSNR_WH_LEN] = +1;
	chips_to_symbols(chips, DVBS2X_VLSNR_HDR_LEN, symbols, 0);
}

void dvbs2x_wh_generate(unsigned int index, int8_t *seq)
{
	vlsnr_generate_pattern(index, seq);
}

/* Reference symbols for frame sync (pre-computed at init) */
static struct dvbs2x_complex vlsnr_ref[DVBS2X_VLSNR_NUM_MODCODS]
	[DVBS2X_VLSNR_WH_LEN];
static struct dvbs2x_complex sync_ref_2048[DVBS2X_VLSNR_NUM_MODCODS][2048];
static struct dvbs2x_complex sync_ref_4096[DVBS2X_VLSNR_NUM_MODCODS][4096];
static struct dvbs2x_complex sync_ref_8192[DVBS2X_VLSNR_NUM_MODCODS][8192];
static int vlsnr_ref_init;
static int sync_fft_ref_init;

static void sync_fft_reference_init(void);

static void do_sync_init(void)
{
	unsigned int mc;

	for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
		int8_t chips[DVBS2X_VLSNR_WH_LEN];
		unsigned int aidx = modcod_to_annex_i[mc];

		vlsnr_generate_pattern(aidx, chips);
		chips_to_symbols(chips, DVBS2X_VLSNR_WH_LEN,
				 vlsnr_ref[mc], 0);
	}
	vlsnr_ref_init = 1;
}

/* Called by dvbs2x_library_init() for thread-safe startup */
void dvbs2x_vlsnr_sync_init(void)
{
	if (!vlsnr_ref_init)
		do_sync_init();
	sync_fft_reference_init();
}

static void sync_fft(struct dvbs2x_complex *data, unsigned int len,
		     int inverse)
{
	unsigned int i, j, width;

	for (i = 1, j = 0; i < len; i++) {
		unsigned int bit = len >> 1;

		while (j & bit) {
			j ^= bit;
			bit >>= 1;
		}
		j ^= bit;
		if (i < j) {
			struct dvbs2x_complex tmp = data[i];

			data[i] = data[j];
			data[j] = tmp;
		}
	}
	for (width = 2; width <= len; width <<= 1) {
		double angle = (inverse ? 2.0 : -2.0) * M_PI /
			(double)width;
		double step_i = cos(angle), step_q = sin(angle);
		unsigned int base;

		for (base = 0; base < len; base += width) {
			double wi = 1.0, wq = 0.0;
			unsigned int half = width >> 1;
			unsigned int n;

			for (n = 0; n < half; n++) {
				struct dvbs2x_complex even = data[base + n];
				struct dvbs2x_complex odd = data[base + n + half];
				double oi = odd.i * wi - odd.q * wq;
				double oq = odd.i * wq + odd.q * wi;
				double next_i;

				data[base + n].i = even.i + oi;
				data[base + n].q = even.q + oq;
				data[base + n + half].i = even.i - oi;
				data[base + n + half].q = even.q - oq;
				next_i = wi * step_i - wq * step_q;
				wq = wi * step_q + wq * step_i;
				wi = next_i;
			}
		}
	}
	if (inverse) {
		for (i = 0; i < len; i++) {
			data[i].i /= (double)len;
			data[i].q /= (double)len;
		}
	}
}

static void sync_fft_reference_init(void)
{
	unsigned int mc, n;

	if (sync_fft_ref_init)
		return;
	for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
		for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
			struct dvbs2x_complex value;

			value.i = vlsnr_ref[mc][DVBS2X_VLSNR_WH_LEN - 1 - n].i;
			value.q = -vlsnr_ref[mc]
				[DVBS2X_VLSNR_WH_LEN - 1 - n].q;
			sync_ref_2048[mc][n] = value;
			sync_ref_4096[mc][n] = value;
			sync_ref_8192[mc][n] = value;
		}
		sync_fft(sync_ref_2048[mc], 2048, 0);
		sync_fft(sync_ref_4096[mc], 4096, 0);
		sync_fft(sync_ref_8192[mc], 8192, 0);
	}
	sync_fft_ref_init = 1;
}

static const struct dvbs2x_complex *sync_fft_reference(unsigned int fft_len,
						       unsigned int modcod)
{
	switch (fft_len) {
	case 2048:
		return sync_ref_2048[modcod];
	case 4096:
		return sync_ref_4096[modcod];
	case 8192:
		return sync_ref_8192[modcod];
	default:
		return NULL;
	}
}

double dvbs2x_vlsnr_header_sync_mode(const struct dvbs2x_complex *symbols,
				     unsigned int len,
				     unsigned int seg_len,
				     unsigned int preferred_modcod,
				     unsigned int *offset,
				     unsigned int *modcod_idx)
{
	struct dvbs2x_complex *input_fft = NULL;
	struct dvbs2x_complex *work = NULL;
	struct dvbs2x_complex *reference = NULL;
	double *energy = NULL;
	double candidate_score[DVBS2X_VLSNR_NUM_MODCODS] = { 0 };
	unsigned int candidate_offset[DVBS2X_VLSNR_NUM_MODCODS] = { 0 };
	double best_corr = -1.0;
	unsigned int best_offset = 0;
	unsigned int best_modcod = 1;
	unsigned int fft_len, positions;
	unsigned int mc, mc_first = 0, mc_last = DVBS2X_VLSNR_NUM_MODCODS;
	unsigned int pos, n;

	if (seg_len == 0 || seg_len > DVBS2X_VLSNR_WH_LEN)
		seg_len = DVBS2X_VLSNR_WH_LEN;
	if (preferred_modcod >= 1 &&
	    preferred_modcod <= DVBS2X_VLSNR_NUM_MODCODS) {
		mc_first = preferred_modcod - 1;
		mc_last = preferred_modcod;
		best_modcod = preferred_modcod;
	}

	/* Ensure references are initialized (lazy fallback) */
	if (!vlsnr_ref_init)
		do_sync_init();
	sync_fft_reference_init();

	if (len < DVBS2X_VLSNR_WH_LEN)
		goto done;
	if (len > UINT_MAX - DVBS2X_VLSNR_WH_LEN + 1)
		goto done;
	fft_len = 1;
	while (fft_len < len + DVBS2X_VLSNR_WH_LEN - 1) {
		if (fft_len > UINT_MAX / 2)
			goto done;
		fft_len <<= 1;
	}
	positions = len - DVBS2X_VLSNR_WH_LEN + 1;
	input_fft = calloc(fft_len, sizeof(*input_fft));
	work = calloc(fft_len, sizeof(*work));
	reference = calloc(fft_len, sizeof(*reference));
	energy = malloc(positions * sizeof(*energy));
	if (!input_fft || !work || !reference || !energy)
		goto done;
	memcpy(input_fft, symbols, len * sizeof(*input_fft));
	sync_fft(input_fft, fft_len, 0);
	energy[0] = 0.0;
	for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++)
		energy[0] += symbols[n].i * symbols[n].i +
			symbols[n].q * symbols[n].q;
	for (pos = 1; pos < positions; pos++) {
		energy[pos] = energy[pos - 1] -
			(symbols[pos - 1].i * symbols[pos - 1].i +
			 symbols[pos - 1].q * symbols[pos - 1].q) +
			(symbols[pos + DVBS2X_VLSNR_WH_LEN - 1].i *
			 symbols[pos + DVBS2X_VLSNR_WH_LEN - 1].i +
			 symbols[pos + DVBS2X_VLSNR_WH_LEN - 1].q *
			 symbols[pos + DVBS2X_VLSNR_WH_LEN - 1].q);
	}
	for (mc = mc_first; mc < mc_last; mc++) {
		const struct dvbs2x_complex *reference_fft;

		reference_fft = sync_fft_reference(fft_len, mc);
		if (!reference_fft) {
			memset(reference, 0, fft_len * sizeof(*reference));
			for (n = 0; n < DVBS2X_VLSNR_WH_LEN; n++) {
				reference[n].i = vlsnr_ref[mc]
					[DVBS2X_VLSNR_WH_LEN - 1 - n].i;
				reference[n].q = -vlsnr_ref[mc]
					[DVBS2X_VLSNR_WH_LEN - 1 - n].q;
			}
			sync_fft(reference, fft_len, 0);
			reference_fft = reference;
		}
		for (n = 0; n < fft_len; n++) {
			work[n].i = input_fft[n].i * reference_fft[n].i -
				input_fft[n].q * reference_fft[n].q;
			work[n].q = input_fft[n].i * reference_fft[n].q +
				input_fft[n].q * reference_fft[n].i;
		}
		sync_fft(work, fft_len, 1);
		for (pos = 0; pos < positions; pos++) {
			double norm, score;
			unsigned int index = pos + DVBS2X_VLSNR_WH_LEN - 1;

			if (energy[pos] <= 0.0)
				continue;
			norm = sqrt((double)DVBS2X_VLSNR_WH_LEN *
				    energy[pos]);
			score = sqrt(work[index].i * work[index].i +
				     work[index].q * work[index].q) / norm;
			if (score > candidate_score[mc]) {
				candidate_score[mc] = score;
				candidate_offset[mc] = pos;
			}
		}
	}
	for (mc = mc_first; mc < mc_last; mc++) {
		const struct dvbs2x_complex *rf = vlsnr_ref[mc];
		double seg_mag = 0.0, norm;
		unsigned int seg_start;

		pos = candidate_offset[mc];
		if (energy[pos] <= 0.0)
			continue;
		norm = sqrt((double)DVBS2X_VLSNR_WH_LEN * energy[pos]);
		for (seg_start = 0; seg_start < DVBS2X_VLSNR_WH_LEN;
		     seg_start += seg_len) {
			unsigned int seg_end = seg_start + seg_len;
			double si = 0.0, sq = 0.0;

			if (seg_end > DVBS2X_VLSNR_WH_LEN)
				seg_end = DVBS2X_VLSNR_WH_LEN;
			for (n = seg_start; n < seg_end; n++) {
				double ri = symbols[pos + n].i;
				double rq = symbols[pos + n].q;

				si += ri * rf[n].i + rq * rf[n].q;
				sq += rq * rf[n].i - ri * rf[n].q;
			}
			seg_mag += sqrt(si * si + sq * sq);
		}
		if (seg_mag / norm > best_corr) {
			best_corr = seg_mag / norm;
			best_offset = pos;
			best_modcod = mc + 1;
		}
	}

done:
	free(input_fft);
	free(work);
	free(reference);
	free(energy);
	if (offset)
		*offset = best_offset;
	if (modcod_idx)
		*modcod_idx = best_modcod;
	return best_corr;
}

double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx)
{
	return dvbs2x_vlsnr_header_sync_mode(symbols, len, seg_len, 0,
					      offset, modcod_idx);
}
