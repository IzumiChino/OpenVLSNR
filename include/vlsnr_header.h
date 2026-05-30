/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Walsh-Hadamard Header
 *
 * The VL-SNR header is a 900-symbol pi/2-BPSK preamble:
 *   [2 zero-pad] [896-bit WH pattern] [2 zero-pad]
 *
 * The 896-bit pattern is constructed from a fixed 16x56 base matrix
 * (clause 5.5.2.5) with rows kept or inverted according to a
 * 16-element Walsh-Hadamard sign sequence (Table 18b) that
 * identifies the MODCOD.
 *
 * Reference: ETSI EN 302 307-2 V1.3.1, clause 5.5.2.5
 */

#ifndef DVBS2X_VLSNR_HEADER_H
#define DVBS2X_VLSNR_HEADER_H

#include "dvbs2x_types.h"

/*
 * dvbs2x_vlsnr_header_generate - Generate VL-SNR header symbols
 * @modcod: MODCOD parameters (determines WH sequence selection)
 * @symbols: output array (DVBS2X_VLSNR_HDR_LEN = 900 symbols)
 *
 * Generates the standard-defined 900-symbol VL-SNR header:
 * 2 zero-pad + 896 WH pattern + 2 zero-pad, pi/2-BPSK modulated.
 */
void dvbs2x_vlsnr_header_generate(const struct dvbs2x_modcod *modcod,
				  struct dvbs2x_complex *symbols);

/*
 * dvbs2x_vlsnr_header_sync - Correlate received signal with headers
 * @symbols: received symbol stream
 * @len: length of symbol stream
 * @seg_len: correlation segment length (divides 896)
 * @offset: detected WH sequence start offset (output)
 * @modcod_idx: detected MODCOD index (output, 1-9)
 *
 * Correlates against the 896-symbol WH portion of the header.
 * The returned offset points to the WH start (2 symbols after
 * the actual header start including zero-padding).
 *
 * Returns correlation peak value (0.0 to ~1.0 normalized).
 */
double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx);

/*
 * dvbs2x_wh_generate - Generate VL-SNR WH chip sequence
 * @index: Annex-I index (0-15)
 * @seq: output sequence (DVBS2X_VLSNR_WH_LEN elements, +1/-1)
 */
void dvbs2x_wh_generate(unsigned int index, int8_t *seq);

#endif /* DVBS2X_VLSNR_HEADER_H */
