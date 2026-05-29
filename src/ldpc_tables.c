// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X VL-SNR LDPC Table Selection
 *
 * Selects the appropriate LDPC parity address table based on
 * the MODCOD parameters (frame length and code rate).
 */

#include "ldpc.h"

const struct dvbs2x_ldpc_table_entry *
dvbs2x_ldpc_get_table(const struct dvbs2x_modcod *modcod,
		      unsigned int *num_groups)
{
	const struct dvbs2x_ldpc_table_entry *table = NULL;
	unsigned int k = modcod->k_ldpc;

	*num_groups = k / DVBS2X_LDPC_PARALLEL;

	switch (modcod->fec_len) {
	case DVBS2X_LDPC_NORMAL:
		/* Rate 2/9, normal frame */
		table = dvbs2x_ldpc_get_table_n64800_r2_9();
		break;
	case DVBS2X_LDPC_MEDIUM:
		switch (modcod->code_rate_den) {
		case 5:
			table = dvbs2x_ldpc_get_table_n32400_r1_5();
			break;
		case 45:
			table = dvbs2x_ldpc_get_table_n32400_r11_45();
			break;
		case 3:
			table = dvbs2x_ldpc_get_table_n32400_r1_3();
			break;
		}
		break;
	case DVBS2X_LDPC_SHORT:
		switch (modcod->code_rate_den) {
		case 5:
			table = dvbs2x_ldpc_get_table_n16200_r1_5();
			break;
		case 45:
			table = dvbs2x_ldpc_get_table_n16200_r11_45();
			break;
		case 15:
			table = dvbs2x_ldpc_get_table_n16200_r4_15();
			break;
		case 3:
			table = dvbs2x_ldpc_get_table_n16200_r1_3();
			break;
		}
		break;
	}

	return table;
}
