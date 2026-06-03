// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Data-Field Layout
 *
 * VL-SNR SET1 and SET2 data/pilot interleaving (gr-dtv physical_cc).
 * Builds a per-symbol map (data vs pilot) that both the modulator and
 * demodulator walk.
 *
 * Geometry per frame (slots = num_data / 90), starting slot_count = 10:
 *   - one "group" pilot block per group, inserted mid-slot when the
 *     running data count reaches a set-specific threshold:
 *       SET1: group<=18 -> 34 syms at 703; group 19..21 -> 36 syms at 702
 *       SET2: group<=9  -> 32 syms at 704; group 10     -> 36 syms at 702
 *   - one 36-symbol "slot-boundary" pilot block every 16 slots (when
 *     slot_count wraps to 0), which advances the group and resets the
 *     running data count.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2
 */

#include "vlsnr_layout.h"
#include <stdlib.h>

static void build_set(uint8_t *map, unsigned int ndata, int set1,
		      unsigned int *out_len, unsigned int *out_pilot)
{
	/*
	 * gr-dtv sizes the slot loop from frame_size, which reserves
	 * EXTRA_PILOT_SYMBOLS for the per-group pilot blocks that replace
	 * data positions: SET1 = 18*34 + 3*36 = 720, SET2 = 9*32 + 36 = 324.
	 */
	unsigned int extra = set1 ? (18 * 34 + 3 * 36) : (9 * 32 + 36);
	int slots = (int)((ndata + extra) / 90);
	int slot_count = 10;
	int group = 0;
	int symbols = 0;
	int j, k, p, x;
	unsigned int out = 0;
	unsigned int dcount = 0;

	for (j = 0; j < slots; j++) {
		for (k = 0; k < 90; k++) {
			int fired = 0;
			int plen = 0;

			map[out++] = 0;		/* data symbol */
			dcount++;
			symbols++;

			if (set1) {
				if (group <= 18 && symbols == 703) {
					plen = 34;
					fired = 1;
				} else if (group >= 19 && group <= 21 &&
					   symbols == 702) {
					plen = 36;
					fired = 1;
				}
			} else {
				if (group <= 9 && symbols == 704) {
					plen = 32;
					fired = 1;
				} else if (group == 10 && symbols == 702) {
					plen = 36;
					fired = 1;
				}
			}

			if (fired) {
				for (p = 0; p < plen; p++)
					map[out++] = 1;	/* group pilots */
				for (x = (k + 1 + plen) - 90; x < 90; x++) {
					map[out++] = 0;
					dcount++;
					symbols++;
				}
				slot_count = (slot_count + 1) % 16;
				j++;
				break;
			}
		}

		slot_count = (slot_count + 1) % 16;
		if (slot_count == 0 && j < slots - 1) {
			group++;
			symbols = 0;
			for (p = 0; p < 36; p++)
				map[out++] = 1;	/* slot-boundary pilots */
		}
	}

	*out_len = out;
	*out_pilot = out - dcount;
}

int dvbs2x_vlsnr_build_layout(const struct dvbs2x_modcod *mc,
			      struct dvbs2x_vlsnr_layout *lay)
{
	unsigned int ndata;
	unsigned int field_len, num_pilot;
	uint8_t *map;

	ndata = dvbs2x_tx_coded_bits(mc);
	if (mc->modulation == DVBS2X_MOD_QPSK)
		ndata /= 2;
	if (mc->has_spread)
		ndata *= 2;

	/* Upper bound: every 90-symbol slot plus a generous pilot margin. */
	map = malloc((size_t)ndata + (size_t)(ndata / 90 + 4) * 36 + 64);
	if (!map)
		return DVBS2X_ERR_NOMEM;

	build_set(map, ndata, mc->set == DVBS2X_VLSNR_SET1,
		  &field_len, &num_pilot);

	lay->is_pilot = map;
	lay->field_len = field_len;
	lay->num_data = ndata;
	lay->num_pilot = num_pilot;
	return DVBS2X_OK;
}

void dvbs2x_vlsnr_free_layout(struct dvbs2x_vlsnr_layout *lay)
{
	free(lay->is_pilot);
	lay->is_pilot = NULL;
	lay->field_len = 0;
	lay->num_data = 0;
	lay->num_pilot = 0;
}
