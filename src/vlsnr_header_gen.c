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
#include <math.h>
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
static int vlsnr_ref_init;

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
}

double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx)
{
	double best_corr = -1.0;
	unsigned int best_offset = 0;
	unsigned int best_modcod = 1;
	unsigned int mc, pos, n;

	if (seg_len == 0 || seg_len > DVBS2X_VLSNR_WH_LEN)
		seg_len = DVBS2X_VLSNR_WH_LEN;

	/* Ensure references are initialized (lazy fallback) */
	if (!vlsnr_ref_init)
		do_sync_init();

	if (len < DVBS2X_VLSNR_WH_LEN)
		goto done;

	/* Slide a correlation window over the input */
	for (pos = 0; pos + DVBS2X_VLSNR_WH_LEN <= len; pos++) {
		for (mc = 0; mc < DVBS2X_VLSNR_NUM_MODCODS; mc++) {
			const struct dvbs2x_complex *rf = vlsnr_ref[mc];
			double seg_mag = 0.0;
			unsigned int seg_start;
			double corr;

			for (seg_start = 0;
			     seg_start < DVBS2X_VLSNR_WH_LEN;
			     seg_start += seg_len) {
				unsigned int seg_end;
				double si = 0.0, sq = 0.0;

				seg_end = seg_start + seg_len;
				if (seg_end > DVBS2X_VLSNR_WH_LEN)
					seg_end = DVBS2X_VLSNR_WH_LEN;

				for (n = seg_start; n < seg_end; n++) {
					double ri, rq;

					ri = symbols[pos + n].i;
					rq = symbols[pos + n].q;
					si += ri * rf[n].i +
					      rq * rf[n].q;
					sq += rq * rf[n].i -
					      ri * rf[n].q;
				}
				seg_mag += sqrt(si * si + sq * sq);
			}

			corr = seg_mag /
			       (double)DVBS2X_VLSNR_WH_LEN;
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
