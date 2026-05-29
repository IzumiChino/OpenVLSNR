// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR BCH Encoder
 *
 * Systematic BCH encoding using LFSR-based polynomial division.
 * The generator polynomial g(x) is computed at init time as the
 * LCM of minimal polynomials of alpha^1 through alpha^(2t).
 *
 * Normal frame:  GF(2^16), t=12, parity = 192 bits
 * Medium frame:  GF(2^15), t=12, parity = 180 bits
 * Short frame:   GF(2^14), t=12, parity = 168 bits
 *
 * Reference: ETSI EN 302 307-1 clause 5.3.1
 *            ETSI EN 302 307-2 for medium frame additions
 */

#include "bch.h"
#include <string.h>
#include <stdlib.h>

/* Primitive polynomials - must match bch_decode.c */
static const uint32_t prim_polys[] = {
	[14] = 0x402B,		/* x^14 + x^5 + x^3 + x + 1 */
	[15] = 0x8003,		/* x^15 + x + 1 */
	[16] = 0x1002D,	/* x^16 + x^5 + x^3 + x^2 + 1 */
};

/*
 * Compute minimal polynomial of alpha^k over GF(2).
 *
 * Method: find the minimum-degree binary polynomial p(x) such that
 * p(alpha^k) = 0. This is done by computing successive powers of
 * alpha^k and finding a linear dependency over GF(2).
 *
 * Returns the polynomial as a bitmask (bit i = coefficient of x^i).
 */
static uint32_t compute_min_poly(unsigned int m, uint32_t prim_poly __attribute__((unused)),
				 unsigned int k, unsigned int *deg,
				 const int *exp_table, unsigned int n)
{
	/*
	 * Compute alpha^(k*0), alpha^(k*1), ..., alpha^(k*m) as
	 * m-bit vectors. Find the first linear dependency.
	 *
	 * We use the fact that alpha^j is represented as an m-bit
	 * integer in our GF tables.
	 */
	unsigned int powers[17];	/* alpha^(k*i) for i=0..m */
	unsigned int matrix[17];	/* row-reduced form */
	unsigned int pivot_col[17];
	unsigned int i, d;
	uint32_t result;

	/* Compute powers: alpha^(k*i) = exp_table[(k*i) % n] */
	for (i = 0; i <= m; i++)
		powers[i] = exp_table[(k * i) % n];

	/*
	 * Gaussian elimination to find dependency.
	 * We want to find coefficients c0, c1, ..., cd (with cd=1)
	 * such that c0*powers[0] + c1*powers[1] + ... + cd*powers[d] = 0
	 * where addition is XOR (GF(2) vector addition).
	 *
	 * Process columns left to right, maintaining row echelon form.
	 */
	memset(matrix, 0, sizeof(matrix));
	memset(pivot_col, 0, sizeof(pivot_col));

	for (d = 0; d <= m; d++) {
		unsigned int vec = powers[d];
		unsigned int row_bits = (1u << d);  /* tracks which powers contribute */

		/* Reduce vec using existing pivots */
		for (i = 0; i < m; i++) {
			if ((vec >> i) & 1) {
				if (pivot_col[i]) {
					vec ^= matrix[i];
					row_bits ^= pivot_col[i];
				}
			}
		}

		if (vec == 0) {
			/* Found dependency: row_bits gives the polynomial */
			result = row_bits;
			*deg = d;
			return result;
		}

		/* No dependency yet; add as new pivot */
		/* Find lowest set bit for pivot position */
		for (i = 0; i < m; i++) {
			if ((vec >> i) & 1) {
				matrix[i] = vec;
				pivot_col[i] = row_bits;
				break;
			}
		}
	}

	/* Should not reach here for valid inputs */
	*deg = m;
	return (1u << m) | 1;
}

/*
 * Multiply two binary polynomials (GF(2)[x]).
 * a has degree deg_a, b has degree deg_b.
 * Result stored in gen_poly array.
 */
static void gf2_poly_mul(uint8_t *result, unsigned int *result_deg,
			  const uint8_t *a, unsigned int deg_a,
			  uint32_t b_packed, unsigned int deg_b)
{
	uint8_t temp[DVBS2X_BCH_MAX_PARITY + 1];
	unsigned int i, j;

	memset(temp, 0, sizeof(temp));

	for (i = 0; i <= deg_a; i++) {
		if (!a[i])
			continue;
		for (j = 0; j <= deg_b; j++) {
			if ((b_packed >> j) & 1)
				temp[i + j] ^= 1;
		}
	}

	*result_deg = deg_a + deg_b;
	memcpy(result, temp, *result_deg + 1);
}

int dvbs2x_bch_encoder_init(struct dvbs2x_bch_encoder *enc,
			    const struct dvbs2x_modcod *modcod)
{
	uint32_t prim_poly;
	unsigned int m;
	unsigned int n;
	uint8_t visited_roots[65536];
	unsigned int gen_deg;
	unsigned int k;
	int *exp_table;
	unsigned int reg, i;

	enc->t = modcod->bch_t;
	enc->k = modcod->k_bch;
	enc->n = modcod->n_bch;
	enc->parity_len = enc->n - enc->k;

	switch (modcod->frame_type) {
	case DVBS2X_FRAME_NORMAL:
		m = 16;
		break;
	case DVBS2X_FRAME_MEDIUM:
		m = 15;
		break;
	case DVBS2X_FRAME_SHORT:
		m = 14;
		break;
	default:
		return -1;
	}

	enc->gf_m = m;
	prim_poly = prim_polys[m];
	n = (1u << m) - 1;

	/* Build GF exp table for minimal polynomial computation */
	exp_table = malloc(2 * n * sizeof(int));
	if (!exp_table)
		return -1;

	reg = 1;
	for (i = 0; i < n; i++) {
		exp_table[i] = reg;
		reg <<= 1;
		if (reg & (1u << m))
			reg ^= prim_poly;
	}
	for (i = n; i < 2 * n; i++)
		exp_table[i] = exp_table[i - n];

	/* Build generator polynomial as LCM of minimal polys */
	memset(enc->gen_poly, 0, sizeof(enc->gen_poly));
	enc->gen_poly[0] = 1;
	gen_deg = 0;

	memset(visited_roots, 0, sizeof(visited_roots));

	for (k = 1; k <= 2 * enc->t; k++) {
		unsigned int conj;
		unsigned int min_deg;
		uint32_t min_poly;

		/* Skip if this root's minimal poly already included */
		conj = k % n;
		if (visited_roots[conj])
			continue;

		/* Compute minimal polynomial of alpha^k */
		min_poly = compute_min_poly(m, prim_poly, k, &min_deg,
					    exp_table, n);

		/* Mark all conjugates as visited */
		conj = k % n;
		while (!visited_roots[conj]) {
			visited_roots[conj] = 1;
			conj = (conj * 2) % n;
		}

		/* Multiply into generator polynomial */
		gf2_poly_mul(enc->gen_poly, &gen_deg,
			     enc->gen_poly, gen_deg,
			     min_poly, min_deg);
	}

	free(exp_table);

	if (gen_deg != enc->parity_len)
		return -1;

	return 0;
}

int dvbs2x_bch_encode(const struct dvbs2x_bch_encoder *enc,
		      const uint8_t *input, uint8_t *output)
{
	uint8_t lfsr[DVBS2X_BCH_MAX_PARITY];
	unsigned int i, j;
	uint8_t fb;

	/* Initialize shift register to zero */
	memset(lfsr, 0, enc->parity_len);

	/* Systematic encoding: shift input through LFSR */
	for (i = 0; i < enc->k; i++) {
		fb = input[i] ^ lfsr[enc->parity_len - 1];

		/* Shift register with feedback */
		for (j = enc->parity_len - 1; j > 0; j--) {
			if (enc->gen_poly[j])
				lfsr[j] = lfsr[j - 1] ^ fb;
			else
				lfsr[j] = lfsr[j - 1];
		}
		lfsr[0] = enc->gen_poly[0] ? fb : 0;
	}

	/* Output: systematic (input) + parity */
	memcpy(output, input, enc->k);
	for (i = 0; i < enc->parity_len; i++)
		output[enc->k + i] = lfsr[enc->parity_len - 1 - i];

	return 0;
}
