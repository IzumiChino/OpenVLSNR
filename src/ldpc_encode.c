// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR LDPC Encoder
 *
 * Accumulator-based encoding for structured IRA-LDPC codes.
 *
 * Algorithm:
 * 1. Initialize parity bits p[0..M-1] = 0 (M = N - K)
 * 2. For each information bit i[m] (m = 0, 1, ..., K-1):
 *    - Determine which group g = m / 360
 *    - Determine position within group: j = m mod 360
 *    - For each address a in table[g]:
 *      p[(a + j*Q) mod M] ^= i[m]
 * 3. Final accumulation: p[i] ^= p[i-1] for i = 1, ..., M-1
 *
 * The codeword is [i[0..K-1], p[0..M-1]] (systematic).
 *
 * Reference: ETSI EN 302 307-1 Annex B/C
 *            ETSI EN 302 307-2 Annex B (VL-SNR codes)
 */

#include "ldpc.h"
#include <string.h>
#include <stdlib.h>

/*
 * Select the appropriate LDPC table based on MODCOD parameters.
 * Uses the shared dvbs2x_ldpc_get_table() from ldpc_tables.c.
 */

int dvbs2x_ldpc_encoder_init(struct dvbs2x_ldpc_encoder *enc,
			     const struct dvbs2x_modcod *modcod)
{
	unsigned int num_groups;
	const struct dvbs2x_ldpc_table_entry *table;

	table = dvbs2x_ldpc_get_table(modcod, &num_groups);
	if (!table)
		return -1;

	enc->code.n = modcod->fec_len;
	enc->code.k = modcod->k_ldpc;
	enc->code.q = modcod->q_ldpc;
	enc->code.num_groups = num_groups;
	enc->code.table = table;
	enc->xp = modcod->xp;
	enc->p_period = modcod->p_period;
	enc->xs = modcod->xs;

	return 0;
}

int dvbs2x_ldpc_encode(const struct dvbs2x_ldpc_encoder *enc,
		       const uint8_t *info, uint8_t *codeword)
{
	unsigned int k = enc->code.k;
	unsigned int n = enc->code.n;
	unsigned int m = n - k;		/* parity length */
	unsigned int q = enc->code.q;
	uint8_t *parity;
	unsigned int group, j, a;
	unsigned int addr;
	unsigned int i;

	/* Copy information bits to output (systematic) */
	memcpy(codeword, info, k);

	/* Initialize parity to zero */
	parity = codeword + k;
	memset(parity, 0, m);

	/* Step 1: Accumulate based on address table */
	for (group = 0; group < enc->code.num_groups; group++) {
		const struct dvbs2x_ldpc_table_entry *entry;

		entry = &enc->code.table[group];

		for (j = 0; j < DVBS2X_LDPC_PARALLEL; j++) {
			unsigned int bit_idx = group * DVBS2X_LDPC_PARALLEL + j;

			if (!info[bit_idx])
				continue;

			/* XOR into parity positions */
			for (a = 0; a < entry->num_addrs; a++) {
				addr = (entry->addrs[a] + j * q) % m;
				parity[addr] ^= 1;
			}
		}
	}

	/* Step 2: Final accumulation (staircase structure) */
	for (i = 1; i < m; i++)
		parity[i] ^= parity[i - 1];

	return 0;
}
