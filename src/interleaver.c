// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR Bit Interleaver
 *
 * For BPSK (1 bit per symbol): no interleaving needed.
 * For QPSK (2 bits per symbol): column-row interleaving with 2 columns.
 *
 * Write bits column-wise into a matrix of (n_ldpc/2) rows x 2 columns,
 * then read row-wise.
 *
 * Reference: ETSI EN 302 307-1 clause 5.3.3
 */

#include "interleaver.h"

void dvbs2x_interleave(const struct dvbs2x_modcod *modcod,
		       const uint8_t *input, uint8_t *output)
{
	unsigned int n;
	unsigned int rows;
	unsigned int cols;
	unsigned int r, c;
	unsigned int idx;

	n = dvbs2x_tx_coded_bits(modcod);

	if (modcod->modulation == DVBS2X_MOD_BPSK) {
		/* No interleaving for BPSK */
		unsigned int i;

		for (i = 0; i < n; i++)
			output[i] = input[i];
		return;
	}

	/* QPSK: 2 columns */
	cols = 2;
	rows = n / cols;

	/*
	 * Write column-wise, read row-wise:
	 * input[col * rows + row] -> output[row * cols + col]
	 */
	idx = 0;
	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			output[idx] = input[c * rows + r];
			idx++;
		}
	}
}

void dvbs2x_deinterleave(const struct dvbs2x_modcod *modcod,
			 const double *input, double *output)
{
	unsigned int n;
	unsigned int rows;
	unsigned int cols;
	unsigned int r, c;
	unsigned int idx;

	n = dvbs2x_tx_coded_bits(modcod);

	if (modcod->modulation == DVBS2X_MOD_BPSK) {
		unsigned int i;

		for (i = 0; i < n; i++)
			output[i] = input[i];
		return;
	}

	/* QPSK: 2 columns - inverse permutation */
	cols = 2;
	rows = n / cols;

	/*
	 * Inverse of interleave:
	 * input[row * cols + col] -> output[col * rows + row]
	 */
	idx = 0;
	for (r = 0; r < rows; r++) {
		for (c = 0; c < cols; c++) {
			output[c * rows + r] = input[idx];
			idx++;
		}
	}
}
