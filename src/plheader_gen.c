// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X PL Header Generation and Recovery
 *
 * PL Header = SOF (26 symbols) + PLS code (64 symbols), pi/2-BPSK.
 *
 * The 64-bit PLS code is the (64,7) bi-orthogonal code of clause 5.5.2.3,
 * built exactly as the on-air encoder does:
 *   - the 7 high bits select rows of the generator g[7];
 *   - the low bit b0 fills the odd output positions as out[2m+1]=out[2m]^b0;
 *   - the whole 64-bit word is XORed with the fixed PL scrambling sequence.
 *
 * pi/2-BPSK uses the standard period-2 diagonal constellation.  For VL-SNR
 * PL headers (PLS code & 0x80) the 64 PLS-code symbols use the shifted pair
 * m_bpsk[(i&1)+2] while the 26 SOF symbols use m_bpsk[i&1], matching the
 * reference physical-layer framer (gr-dtv dvbs2_physical_cc).
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.2, ETSI EN 302 307-2 (VL-SNR).
 */

#include "plheader.h"
#include <string.h>

#define M_SQRT1_2_F 0.70710678118654752440

/*
 * SOF sequence (26 bits), ETSI EN 302 307-1 Table 11 (hex 18D2E82),
 * identical to the reference framer's ph_sync_seq[26].
 */
static const uint8_t sof_sequence[DVBS2X_SOF_LEN] = {
	0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0,
	1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0
};

/*
 * PLS (64,7) code generator rows and the PL scrambling sequence,
 * from ETSI EN 302 307-1 clause 5.5.2.4 (== gr-dtv g[7]/ph_scram_tab[64]).
 */
static const uint32_t pls_gen[7] = {
	0x90AC2DDD, 0x55555555, 0x33333333, 0x0F0F0F0F,
	0x00FF00FF, 0x0000FFFF, 0xFFFFFFFF
};

static const uint8_t pls_scram[DVBS2X_PLSC_LEN] = {
	0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0,
	0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0
};

/*
 * pi/2-BPSK PL-header constellation m_bpsk[phase][bit] = {I, Q}.
 *   phase 0/1: even/odd SOF symbols (standard period-2 diagonals)
 *   phase 2/3: even/odd VL-SNR PLS-code symbols (shifted pair)
 */
static const double pl_const[4][2][2] = {
	{ {  M_SQRT1_2_F,  M_SQRT1_2_F }, { -M_SQRT1_2_F, -M_SQRT1_2_F } },
	{ { -M_SQRT1_2_F,  M_SQRT1_2_F }, {  M_SQRT1_2_F, -M_SQRT1_2_F } },
	{ { -M_SQRT1_2_F,  M_SQRT1_2_F }, {  M_SQRT1_2_F, -M_SQRT1_2_F } },
	{ { -M_SQRT1_2_F, -M_SQRT1_2_F }, {  M_SQRT1_2_F,  M_SQRT1_2_F } },
};

const uint8_t *dvbs2x_plheader_get_sof(void)
{
	return sof_sequence;
}

/*
 * Encode an 8-bit PLS code into the 64-bit bi-orthogonal codeword.
 * @code: PL signalling code (e.g. 129 = VL-SNR Set 1, 131 = Set 2)
 * @encoded: 64 output bits
 */
static void pls_encode(unsigned int code, uint8_t *encoded)
{
	uint32_t temp = 0, bit;
	unsigned int m;

	if (code & 0x80)
		temp ^= pls_gen[0];
	if (code & 0x40)
		temp ^= pls_gen[1];
	if (code & 0x20)
		temp ^= pls_gen[2];
	if (code & 0x10)
		temp ^= pls_gen[3];
	if (code & 0x08)
		temp ^= pls_gen[4];
	if (code & 0x04)
		temp ^= pls_gen[5];
	if (code & 0x02)
		temp ^= pls_gen[6];

	bit = 0x80000000;
	for (m = 0; m < 32; m++) {
		encoded[m * 2] = (temp & bit) ? 1 : 0;
		encoded[m * 2 + 1] = encoded[m * 2] ^ (code & 0x01);
		bit >>= 1;
	}
	for (m = 0; m < DVBS2X_PLSC_LEN; m++)
		encoded[m] ^= pls_scram[m];
}

void dvbs2x_plheader_generate(unsigned int pls_code,
			      struct dvbs2x_complex *symbols)
{
	uint8_t all_bits[DVBS2X_PLHEADER_LEN];
	int vlsnr = (pls_code & 0x80) != 0;
	unsigned int n;

	/* SOF bits + 64 encoded PLS-code bits (full 8-bit code) */
	memcpy(all_bits, sof_sequence, DVBS2X_SOF_LEN);
	pls_encode(pls_code & 0xFF, all_bits + DVBS2X_SOF_LEN);

	for (n = 0; n < DVBS2X_PLHEADER_LEN; n++) {
		unsigned int b = all_bits[n] & 1;
		unsigned int phase;

		if (n < DVBS2X_SOF_LEN)
			phase = n & 1;			/* SOF: m_bpsk[i&1] */
		else
			phase = (n & 1) + (vlsnr ? 2 : 0);

		symbols[n].i = pl_const[phase][b][0];
		symbols[n].q = pl_const[phase][b][1];
	}
}

int dvbs2x_plheader_recover(const struct dvbs2x_complex *symbols,
			    unsigned int *pls_code)
{
	double plsc_soft[DVBS2X_PLSC_LEN];
	double best_corr;
	unsigned int best_code, code, n;

	/*
	 * Soft de-map of the 64 PLS-code symbols.  b=0 and b=1 map to
	 * antipodal points, so the per-symbol soft value is the correlation
	 * with the b=0 reference; this library only emits VL-SNR PL headers,
	 * whose PLS-code symbols use the shifted pair m_bpsk[(i&1)+2].
	 */
	for (n = 0; n < DVBS2X_PLSC_LEN; n++) {
		unsigned int i = DVBS2X_SOF_LEN + n;
		unsigned int phase = (i & 1) + 2;

		plsc_soft[n] = symbols[i].i * pl_const[phase][0][0] +
			       symbols[i].q * pl_const[phase][0][1];
	}

	/* ML decode: pick the 8-bit code whose codeword correlates best */
	best_corr = -1e30;
	best_code = 0;
	for (code = 0; code < 256; code++) {
		uint8_t enc[DVBS2X_PLSC_LEN];
		double corr = 0.0;

		pls_encode(code, enc);
		for (n = 0; n < DVBS2X_PLSC_LEN; n++)
			corr += plsc_soft[n] * (enc[n] ? -1.0 : 1.0);

		if (corr > best_corr) {
			best_corr = corr;
			best_code = code;
		}
	}

	/* Return the low 7 bits (MODCOD/type field) of the recovered code */
	*pls_code = best_code & 0x7F;
	return 0;
}
