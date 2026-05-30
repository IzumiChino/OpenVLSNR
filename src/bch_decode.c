// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR BCH Decoder
 *
 * Decoding algorithm:
 * 1. Syndrome computation
 * 2. Berlekamp-Massey algorithm for error locator polynomial
 * 3. Chien search for error locations
 * 4. Error correction
 *
 * Reference: ETSI EN 302 307-1 clause 5.3.1
 */

#include "bch.h"
#include <string.h>

/* Maximum GF field order we support */
#define GF_MAX_M	16
#define GF_MAX_N	((1 << GF_MAX_M) - 1)

/* GF(2^m) arithmetic tables */
struct gf_tables {
	unsigned int	m;
	unsigned int	n;		/* 2^m - 1 */
	uint32_t	prim_poly;
	int		exp_table[2 * (1 << GF_MAX_M)];
	int		log_table[1 << GF_MAX_M];
};

/* Primitive polynomials for GF(2^m) */
static const uint32_t gf_prim_polys[] = {
	[14] = 0x402B,		/* x^14 + x^5 + x^3 + x + 1 */
	[15] = 0x8003,		/* x^15 + x + 1 */
	[16] = 0x1002D,	/* x^16 + x^5 + x^3 + x^2 + 1 */
};

static void gf_init(struct gf_tables *gf, unsigned int m)
{
	unsigned int i;
	uint32_t reg;

	gf->m = m;
	gf->n = (1u << m) - 1;
	gf->prim_poly = gf_prim_polys[m];

	/* Build exp and log tables */
	reg = 1;
	for (i = 0; i < gf->n; i++) {
		gf->exp_table[i] = reg;
		gf->log_table[reg] = i;
		reg <<= 1;
		if (reg & (1u << m))
			reg ^= gf->prim_poly;
	}

	/* Extend exp table for easy modular access */
	for (i = gf->n; i < 2 * gf->n; i++)
		gf->exp_table[i] = gf->exp_table[i - gf->n];

	gf->log_table[0] = -1;	/* log(0) undefined */
}

static inline int gf_mul(const struct gf_tables *gf, int a, int b)
{
	if (a == 0 || b == 0)
		return 0;
	return gf->exp_table[gf->log_table[a] + gf->log_table[b]];
}

static inline int gf_add(int a, int b)
{
	return a ^ b;
}

static inline int gf_inv(const struct gf_tables *gf, int a)
{
	if (a == 0)
		return 0;
	return gf->exp_table[gf->n - gf->log_table[a]];
}

int dvbs2x_bch_decoder_init(struct dvbs2x_bch_decoder *dec,
			    const struct dvbs2x_modcod *modcod)
{
	dec->t = modcod->bch_t;
	dec->k = modcod->k_bch;
	dec->n = modcod->n_bch;
	dec->parity_len = dec->n - dec->k;

	switch (modcod->frame_type) {
	case DVBS2X_FRAME_NORMAL:
		dec->gf_m = 16;
		break;
	case DVBS2X_FRAME_MEDIUM:
		dec->gf_m = 15;
		break;
	case DVBS2X_FRAME_SHORT:
		dec->gf_m = 14;
		break;
	default:
		return -1;
	}

	return 0;
}

int dvbs2x_bch_decode(const struct dvbs2x_bch_decoder *dec,
		      uint8_t *codeword)
{
	struct gf_tables gf;
	int syndromes[2 * 12 + 1];	/* max 2t syndromes */
	int sigma[2 * 12 + 1];		/* error locator polynomial */
	int sigma_new[2 * 12 + 1];
	int error_locs[12];		/* max t error locations */
	unsigned int num_errors;
	unsigned int i, j, n;
	int d;
	int l, l_new;
	int b[2 * 12 + 1];

	gf_init(&gf, dec->gf_m);

	/*
	 * Step 1: Compute syndromes S1..S2t
	 *
	 * The codeword is stored MSB-first: codeword[0] corresponds to
	 * x^(n-1), codeword[n-1] corresponds to x^0.
	 * So bit at position j corresponds to x^(n-1-j).
	 */
	for (i = 1; i <= 2 * dec->t; i++) {
		syndromes[i] = 0;
		for (n = 0; n < dec->n; n++) {
			if (codeword[n])
				syndromes[i] = gf_add(syndromes[i],
					gf.exp_table[(i * (dec->n - 1 - n)) %
						     gf.n]);
		}
	}

	/* Check if all syndromes are zero (no errors) */
	num_errors = 0;
	for (i = 1; i <= 2 * dec->t; i++) {
		if (syndromes[i] != 0) {
			num_errors = 1;
			break;
		}
	}
	if (!num_errors)
		return 0;

	/* Step 2: Berlekamp-Massey algorithm */
	memset(sigma, 0, sizeof(sigma));
	memset(b, 0, sizeof(b));
	sigma[0] = 1;
	b[0] = 1;
	l = 0;

	for (n = 1; n <= 2 * dec->t; n++) {
		/* Compute discrepancy */
		d = syndromes[n];
		for (i = 1; (int)i <= l; i++)
			d = gf_add(d, gf_mul(&gf, sigma[i], syndromes[n - i]));

		if (d == 0) {
			/* Shift b(x) -> x*b(x) */
			for (i = 2 * dec->t; i > 0; i--)
				b[i] = b[i - 1];
			b[0] = 0;
		} else {
			/* sigma_new(x) = sigma(x) - d * x * b(x) */
			memcpy(sigma_new, sigma, sizeof(sigma));
			for (i = 0; i <= 2 * dec->t - 1; i++) {
				if (b[i] != 0) {
					unsigned int v;

					v = gf_mul(&gf, d, b[i]);
					sigma_new[i + 1] =
						gf_add(sigma_new[i + 1], v);
				}
			}

			if (2 * l <= (int)n - 1) {
				int d_inv = gf_inv(&gf, d);

				l_new = n - l;
				for (i = 0; i <= 2 * dec->t; i++)
					b[i] = gf_mul(&gf, sigma[i], d_inv);
				l = l_new;
			} else {
				/* Shift b(x) -> x*b(x) */
				for (i = 2 * dec->t; i > 0; i--)
					b[i] = b[i - 1];
				b[0] = 0;
			}

			memcpy(sigma, sigma_new, sizeof(sigma));
		}
	}

	num_errors = l;
	if (num_errors > dec->t)
		return -1;	/* Uncorrectable */

	/* Step 3: Chien search for error locations */
	j = 0;
	for (i = 0; i < gf.n; i++) {
		int eval = 0;
		unsigned int k;

		/* Evaluate sigma(alpha^i) */
		eval = sigma[0];
		for (k = 1; k <= num_errors; k++) {
			if (sigma[k] != 0)
				eval = gf_add(eval,
					gf.exp_table[(gf.log_table[sigma[k]] +
						      k * i) % gf.n]);
		}

		if (eval == 0) {
			/*
			 * alpha^i is a root of sigma(x).
			 * Error locator root alpha^i means error at
			 * polynomial position (n-1-i) mod gf.n,
			 * which maps to array index i (since array[j]
			 * corresponds to x^(n-1-j)).
			 *
			 * Actually: if sigma(alpha^i) = 0, the error
			 * is at position alpha^(-i) = alpha^(gf.n - i).
			 * In our array, polynomial position p maps to
			 * array index (dec->n - 1 - p).
			 */
			unsigned int poly_pos = (gf.n - i) % gf.n;
			unsigned int arr_pos;

			if (poly_pos >= dec->n)
				continue;
			arr_pos = dec->n - 1 - poly_pos;
			error_locs[j] = arr_pos;
			j++;
			if (j >= num_errors)
				break;
		}
	}

	if (j != num_errors)
		return -1;	/* Could not find all error locations */

	/* Step 4: Correct errors */
	for (i = 0; i < num_errors; i++)
		codeword[error_locs[i]] ^= 1;

	return num_errors;
}
