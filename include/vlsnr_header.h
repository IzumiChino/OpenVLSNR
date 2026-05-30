/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Walsh-Hadamard Header
 *
 * The VL-SNR header is a 896-symbol Walsh-Hadamard encoded sequence
 * that conveys the MODCOD information and enables frame synchronization
 * at very low SNR (down to -10 dB Es/N0).
 *
 * Structure: 896 pi/2-BPSK Walsh-Hadamard symbols.  The rotation
 * index matches the receiver reference so the header doubles as a
 * data-aided carrier-recovery preamble.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2.5
 */

#ifndef DVBS2X_VLSNR_HEADER_H
#define DVBS2X_VLSNR_HEADER_H

#include "dvbs2x_types.h"

/* Walsh-Hadamard matrix order (2^10 = 1024, truncated to 896) */
#define DVBS2X_WH_ORDER	1024

/*
 * dvbs2x_vlsnr_header_generate - Generate VL-SNR header symbols
 * @modcod: MODCOD parameters (determines WH sequence selection)
 * @symbols: output array (DVBS2X_VLSNR_HDR_LEN symbols)
 *
 * Generates 896 PN-covered Walsh-Hadamard symbols, pi/2-BPSK modulated.
 */
void dvbs2x_vlsnr_header_generate(const struct dvbs2x_modcod *modcod,
				  struct dvbs2x_complex *symbols);

/*
 * dvbs2x_vlsnr_header_sync - Correlate received signal with WH headers
 * @symbols: received symbol stream
 * @len: length of symbol stream
 * @seg_len: correlation segment length (multiple of 896)
 * @offset: detected frame start offset (output)
 * @modcod_idx: detected MODCOD index (output, 1-9)
 *
 * Performs segment-coherent correlation to detect VL-SNR frame
 * boundaries and identify the MODCOD.
 *
 * Returns correlation peak value (0.0 to 1.0 normalized).
 */
double dvbs2x_vlsnr_header_sync(const struct dvbs2x_complex *symbols,
				unsigned int len,
				unsigned int seg_len,
				unsigned int *offset,
				unsigned int *modcod_idx);

/*
 * dvbs2x_wh_generate - Generate Walsh-Hadamard sequence
 * @index: sequence index (row of WH matrix)
 * @seq: output sequence (DVBS2X_VLSNR_WH_LEN elements, +1/-1)
 */
void dvbs2x_wh_generate(unsigned int index, int8_t *seq);

#endif /* DVBS2X_VLSNR_HEADER_H */
