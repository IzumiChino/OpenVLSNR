/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR PL Header Generation and Recovery
 *
 * PL Header = SOF (26 symbols) + PLS Code (64 symbols) = 90 symbols
 *
 * SOF: fixed known sequence for frame synchronization
 * PLS Code: (64,7) bi-orthogonal code carrying MODCOD + frame type
 *
 * All symbols are pi/2-BPSK modulated.
 *
 * Reference: ETSI EN 302 307-1 clause 5.5.2
 */

#ifndef DVBS2X_PLHEADER_H
#define DVBS2X_PLHEADER_H

#include "dvbs2x_types.h"

/*
 * dvbs2x_plheader_generate - Generate PL header symbols
 * @pls_code: PLS decimal code (129 or 131 for VL-SNR)
 * @symbols: output array of DVBS2X_PLHEADER_LEN complex symbols
 *
 * Generates SOF + encoded PLS code, pi/2-BPSK modulated.
 */
void dvbs2x_plheader_generate(unsigned int pls_code,
			      struct dvbs2x_complex *symbols);

/*
 * dvbs2x_plheader_recover - Recover PLS code from received header
 * @symbols: received DVBS2X_PLHEADER_LEN complex symbols
 * @pls_code: decoded PLS decimal code output
 *
 * Uses ML decoding of (64,7) bi-orthogonal code.
 * Returns 0 on success, -1 on failure.
 */
int dvbs2x_plheader_recover(const struct dvbs2x_complex *symbols,
			    unsigned int *pls_code);

/*
 * dvbs2x_plheader_get_sof - Get SOF reference sequence
 *
 * Returns pointer to 26-element array of SOF bits.
 */
const uint8_t *dvbs2x_plheader_get_sof(void);

#endif /* DVBS2X_PLHEADER_H */
