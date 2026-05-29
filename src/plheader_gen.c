// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X PL Header Generation and Recovery
 *
 * PL Header = SOF (26 symbols) + PLS Code (64 symbols)
 *
 * SOF: fixed 26-bit sequence, pi/2-BPSK modulated
 * PLS Code: 7 information bits encoded with (64,7) bi-orthogonal code
 *
 * The (64,7) code is constructed from a (32,6) first-order Reed-Muller
 * code with an additional overall parity bit selecting between the
 * code and its complement.
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.2
 */

#include "plheader.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * SOF sequence (26 bits) from ETSI EN 302 307-1 Table 11
 * Hex: 18D2E82 (MSB first, 26 bits)
 * Binary: 01 1000 1101 0010 1110 1000 0010
 */
static const uint8_t sof_sequence[DVBS2X_SOF_LEN] = {
	0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0,
	1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0
};

/*
 * First-order Reed-Muller (32,6) generator matrix rows.
 * G = [g0; g1; g2; g3; g4; g5] where each gi is 32 bits.
 * Row 0 is all-ones (repetition), rows 1-5 are Walsh functions.
 */
static const uint32_t rm_generator[6] = {
	0xFFFFFFFF,	/* all ones */
	0xAAAAAAAA,	/* alternating 10 */
	0xCCCCCCCC,	/* alternating 1100 */
	0xF0F0F0F0,	/* alternating 11110000 */
	0xFF00FF00,	/* alternating 8 */
	0xFFFF0000,	/* first half 1, second half 0 */
};

const uint8_t *dvbs2x_plheader_get_sof(void)
{
	return sof_sequence;
}

/*
 * Encode PLS code using (64,7) bi-orthogonal code.
 * Input: 7-bit PLS decimal code (bits b6..b0)
 * Output: 64 encoded bits
 *
 * b6..b1 select a (32,6) RM codeword
 * b0 determines if the 64-bit output uses the codeword or its complement
 * The 64-bit code = [codeword, codeword XOR b0_mask]
 */
static void pls_encode(unsigned int pls_code, uint8_t *encoded)
{
	uint32_t codeword = 0;
	unsigned int i;
	uint8_t b0;

	/* Extract bits: b6 is MSB of the 7-bit code */
	b0 = pls_code & 1;

	/* Generate (32,6) RM codeword from bits b6..b1 */
	for (i = 0; i < 6; i++) {
		if ((pls_code >> (i + 1)) & 1)
			codeword ^= rm_generator[i];
	}

	/* First 32 bits: the codeword */
	for (i = 0; i < 32; i++)
		encoded[i] = (codeword >> (31 - i)) & 1;

	/*
	 * Second 32 bits: complement pattern
	 * Even-indexed output bits = codeword bits
	 * Odd-indexed output bits = codeword bits XOR b0
	 */
	for (i = 0; i < 32; i++) {
		uint8_t bit = (codeword >> (31 - i)) & 1;

		encoded[32 + i] = bit ^ b0;
	}
}

void dvbs2x_plheader_generate(unsigned int pls_code,
			      struct dvbs2x_complex *symbols)
{
	uint8_t plsc_bits[DVBS2X_PLSC_LEN];
	uint8_t all_bits[DVBS2X_PLHEADER_LEN];
	unsigned int n;
	double val, angle;

	/* Combine SOF + encoded PLS code (lower 7 bits only) */
	memcpy(all_bits, sof_sequence, DVBS2X_SOF_LEN);
	pls_encode(pls_code & 0x7F, plsc_bits);
	memcpy(all_bits + DVBS2X_SOF_LEN, plsc_bits, DVBS2X_PLSC_LEN);

	/* pi/2-BPSK modulation */
	for (n = 0; n < DVBS2X_PLHEADER_LEN; n++) {
		val = all_bits[n] ? -1.0 : 1.0;
		angle = (double)(n % 4) * M_PI / 2.0;
		symbols[n].i = val * cos(angle);
		symbols[n].q = val * sin(angle);
	}
}

int dvbs2x_plheader_recover(const struct dvbs2x_complex *symbols,
			    unsigned int *pls_code)
{
	double derot[DVBS2X_PLHEADER_LEN];
	double plsc_soft[DVBS2X_PLSC_LEN];
	unsigned int n;
	double angle, cos_a, sin_a;
	double best_corr;
	unsigned int best_code;
	unsigned int code;

	/* De-rotate pi/2-BPSK */
	for (n = 0; n < DVBS2X_PLHEADER_LEN; n++) {
		angle = (double)(n % 4) * M_PI / 2.0;
		cos_a = cos(angle);
		sin_a = sin(angle);
		derot[n] = symbols[n].i * cos_a + symbols[n].q * sin_a;
	}

	/* Extract PLS code soft values */
	for (n = 0; n < DVBS2X_PLSC_LEN; n++)
		plsc_soft[n] = derot[DVBS2X_SOF_LEN + n];

	/*
	 * ML decoding: correlate with all 128 possible codewords
	 * (7 bits -> 128 possibilities)
	 */
	best_corr = -1e30;
	best_code = 0;

	for (code = 0; code < 128; code++) {
		uint8_t encoded[DVBS2X_PLSC_LEN];
		double corr = 0.0;

		pls_encode(code, encoded);

		for (n = 0; n < DVBS2X_PLSC_LEN; n++) {
			double ref = encoded[n] ? -1.0 : 1.0;

			corr += plsc_soft[n] * ref;
		}

		if (corr > best_corr) {
			best_corr = corr;
			best_code = code;
		}
	}

	/*
	 * Return the 7-bit decoded value. The caller must add the
	 * high bit (0x80) for DVB-S2X VL-SNR codes based on context
	 * (presence of VL-SNR header following the PL header).
	 *
	 * For VL-SNR: PLS decimal code = decoded_7bit | 0x80
	 * For regular DVB-S2/S2X: PLS decimal code = decoded_7bit
	 */
	*pls_code = best_code;
	return 0;
}
