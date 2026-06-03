/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Pilot Block Insertion and Extraction
 *
 * Pilot positions follow the data-field layout (see vlsnr_layout.h).
 * Each pilot symbol is (1+j)/sqrt(2) before PL scrambling.
 *
 * Reference: ETSI EN 302 307-2 clause 5.5.2
 */

#ifndef DVBS2X_PILOT_H
#define DVBS2X_PILOT_H

#include "dvbs2x_types.h"
#include "vlsnr_layout.h"

/* Pilot symbol value before scrambling: m_bpsk[0][0] = (1+j)/sqrt(2) */
#define DVBS2X_PILOT_I	0.70710678118654752440
#define DVBS2X_PILOT_Q	0.70710678118654752440

/*
 * dvbs2x_pilot_insert - Interleave data symbols with pilot blocks
 * @data: payload symbols (lay->num_data of them)
 * @lay: data-field layout
 * @output: output data field (lay->field_len symbols)
 *
 * Returns the number of symbols written (lay->field_len).
 */
unsigned int dvbs2x_pilot_insert(const struct dvbs2x_complex *data,
				 const struct dvbs2x_vlsnr_layout *lay,
				 struct dvbs2x_complex *output);

/*
 * dvbs2x_pilot_extract - Separate payload symbols from a received field
 * @field: received data field (lay->field_len symbols)
 * @lay: data-field layout
 * @data: output payload symbols (lay->num_data of them)
 * @pilots: output pilot symbols (lay->num_pilot of them; may be NULL)
 *
 * Returns the number of payload symbols extracted (lay->num_data).
 */
unsigned int dvbs2x_pilot_extract(const struct dvbs2x_complex *field,
				  const struct dvbs2x_vlsnr_layout *lay,
				  struct dvbs2x_complex *data,
				  struct dvbs2x_complex *pilots);

#endif /* DVBS2X_PILOT_H */
