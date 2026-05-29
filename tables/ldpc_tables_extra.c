// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X LDPC Parity Address Tables (continued)
 *
 * Stub tables for remaining code rates.
 * TODO: Fill with actual values from ETSI EN 302 307-2 Annex B.
 */

#include "ldpc.h"

/* Rate 11/45, Medium frame (32400 bits), K=7920, M=24480, Q=68 */
static const struct dvbs2x_ldpc_table_entry table_n32400_r11_45[22] = {
	{ .num_addrs = 3, .addrs = { 0, 1000, 5000 } },
	{ .num_addrs = 3, .addrs = { 100, 2000, 10000 } },
	{ .num_addrs = 3, .addrs = { 200, 3000, 15000 } },
	{ .num_addrs = 3, .addrs = { 300, 4000, 20000 } },
	{ .num_addrs = 3, .addrs = { 400, 5000, 24000 } },
	{ .num_addrs = 3, .addrs = { 500, 6000, 1000 } },
	{ .num_addrs = 3, .addrs = { 600, 7000, 3000 } },
	{ .num_addrs = 3, .addrs = { 700, 8000, 5000 } },
	{ .num_addrs = 3, .addrs = { 800, 9000, 7000 } },
	{ .num_addrs = 3, .addrs = { 900, 10000, 9000 } },
	{ .num_addrs = 3, .addrs = { 1000, 11000, 11000 } },
	{ .num_addrs = 3, .addrs = { 1100, 12000, 13000 } },
	{ .num_addrs = 3, .addrs = { 1200, 13000, 15000 } },
	{ .num_addrs = 3, .addrs = { 1300, 14000, 17000 } },
	{ .num_addrs = 3, .addrs = { 1400, 15000, 19000 } },
	{ .num_addrs = 3, .addrs = { 1500, 16000, 21000 } },
	{ .num_addrs = 3, .addrs = { 1600, 17000, 23000 } },
	{ .num_addrs = 3, .addrs = { 1700, 18000, 24000 } },
	{ .num_addrs = 3, .addrs = { 1800, 19000, 2000 } },
	{ .num_addrs = 3, .addrs = { 1900, 20000, 4000 } },
	{ .num_addrs = 3, .addrs = { 2000, 21000, 6000 } },
	{ .num_addrs = 3, .addrs = { 2100, 22000, 8000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r11_45(void)
{
	return table_n32400_r11_45;
}

/* Rate 1/3, Medium frame (32400 bits), K=10800, M=21600, Q=60 */
static const struct dvbs2x_ldpc_table_entry table_n32400_r1_3[30] = {
	{ .num_addrs = 3, .addrs = { 0, 1000, 5000 } },
	{ .num_addrs = 3, .addrs = { 100, 2000, 10000 } },
	{ .num_addrs = 3, .addrs = { 200, 3000, 15000 } },
	{ .num_addrs = 3, .addrs = { 300, 4000, 20000 } },
	{ .num_addrs = 3, .addrs = { 400, 5000, 21000 } },
	{ .num_addrs = 3, .addrs = { 500, 6000, 1000 } },
	{ .num_addrs = 3, .addrs = { 600, 7000, 3000 } },
	{ .num_addrs = 3, .addrs = { 700, 8000, 5000 } },
	{ .num_addrs = 3, .addrs = { 800, 9000, 7000 } },
	{ .num_addrs = 3, .addrs = { 900, 10000, 9000 } },
	{ .num_addrs = 3, .addrs = { 1000, 11000, 11000 } },
	{ .num_addrs = 3, .addrs = { 1100, 12000, 13000 } },
	{ .num_addrs = 3, .addrs = { 1200, 13000, 15000 } },
	{ .num_addrs = 3, .addrs = { 1300, 14000, 17000 } },
	{ .num_addrs = 3, .addrs = { 1400, 15000, 19000 } },
	{ .num_addrs = 3, .addrs = { 1500, 16000, 21000 } },
	{ .num_addrs = 3, .addrs = { 1600, 17000, 2000 } },
	{ .num_addrs = 3, .addrs = { 1700, 18000, 4000 } },
	{ .num_addrs = 3, .addrs = { 1800, 19000, 6000 } },
	{ .num_addrs = 3, .addrs = { 1900, 20000, 8000 } },
	{ .num_addrs = 3, .addrs = { 2000, 21000, 10000 } },
	{ .num_addrs = 3, .addrs = { 2100, 1000, 12000 } },
	{ .num_addrs = 3, .addrs = { 2200, 2000, 14000 } },
	{ .num_addrs = 3, .addrs = { 2300, 3000, 16000 } },
	{ .num_addrs = 3, .addrs = { 2400, 4000, 18000 } },
	{ .num_addrs = 3, .addrs = { 2500, 5000, 20000 } },
	{ .num_addrs = 3, .addrs = { 2600, 6000, 21000 } },
	{ .num_addrs = 3, .addrs = { 2700, 7000, 1000 } },
	{ .num_addrs = 3, .addrs = { 2800, 8000, 3000 } },
	{ .num_addrs = 3, .addrs = { 2900, 9000, 5000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r1_3(void)
{
	return table_n32400_r1_3;
}

/* Rate 1/5, Short frame (16200 bits), K=3240, M=12960, Q=36 */
static const struct dvbs2x_ldpc_table_entry table_n16200_r1_5[9] = {
	{ .num_addrs = 3, .addrs = { 0, 500, 3000 } },
	{ .num_addrs = 3, .addrs = { 100, 1000, 6000 } },
	{ .num_addrs = 3, .addrs = { 200, 1500, 9000 } },
	{ .num_addrs = 3, .addrs = { 300, 2000, 12000 } },
	{ .num_addrs = 3, .addrs = { 400, 2500, 1000 } },
	{ .num_addrs = 3, .addrs = { 500, 3000, 4000 } },
	{ .num_addrs = 3, .addrs = { 600, 3500, 7000 } },
	{ .num_addrs = 3, .addrs = { 700, 4000, 10000 } },
	{ .num_addrs = 3, .addrs = { 800, 4500, 12500 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r1_5(void)
{
	return table_n16200_r1_5;
}

/* Rate 11/45, Short frame (16200 bits), K=3960, M=12240, Q=34 */
static const struct dvbs2x_ldpc_table_entry table_n16200_r11_45[11] = {
	{ .num_addrs = 3, .addrs = { 0, 500, 3000 } },
	{ .num_addrs = 3, .addrs = { 100, 1000, 6000 } },
	{ .num_addrs = 3, .addrs = { 200, 1500, 9000 } },
	{ .num_addrs = 3, .addrs = { 300, 2000, 12000 } },
	{ .num_addrs = 3, .addrs = { 400, 2500, 1000 } },
	{ .num_addrs = 3, .addrs = { 500, 3000, 4000 } },
	{ .num_addrs = 3, .addrs = { 600, 3500, 7000 } },
	{ .num_addrs = 3, .addrs = { 700, 4000, 10000 } },
	{ .num_addrs = 3, .addrs = { 800, 4500, 11000 } },
	{ .num_addrs = 3, .addrs = { 900, 5000, 12000 } },
	{ .num_addrs = 3, .addrs = { 1000, 5500, 2000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r11_45(void)
{
	return table_n16200_r11_45;
}

/* Rate 4/15, Short frame (16200 bits), K=4320, M=11880, Q=33 */
static const struct dvbs2x_ldpc_table_entry table_n16200_r4_15[12] = {
	{ .num_addrs = 3, .addrs = { 0, 500, 3000 } },
	{ .num_addrs = 3, .addrs = { 100, 1000, 6000 } },
	{ .num_addrs = 3, .addrs = { 200, 1500, 9000 } },
	{ .num_addrs = 3, .addrs = { 300, 2000, 11000 } },
	{ .num_addrs = 3, .addrs = { 400, 2500, 1000 } },
	{ .num_addrs = 3, .addrs = { 500, 3000, 4000 } },
	{ .num_addrs = 3, .addrs = { 600, 3500, 7000 } },
	{ .num_addrs = 3, .addrs = { 700, 4000, 10000 } },
	{ .num_addrs = 3, .addrs = { 800, 4500, 11500 } },
	{ .num_addrs = 3, .addrs = { 900, 5000, 2000 } },
	{ .num_addrs = 3, .addrs = { 1000, 5500, 5000 } },
	{ .num_addrs = 3, .addrs = { 1100, 6000, 8000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r4_15(void)
{
	return table_n16200_r4_15;
}

/* Rate 1/3, Short frame (16200 bits), K=5400, M=10800, Q=30 */
static const struct dvbs2x_ldpc_table_entry table_n16200_r1_3[15] = {
	{ .num_addrs = 3, .addrs = { 0, 500, 3000 } },
	{ .num_addrs = 3, .addrs = { 100, 1000, 6000 } },
	{ .num_addrs = 3, .addrs = { 200, 1500, 9000 } },
	{ .num_addrs = 3, .addrs = { 300, 2000, 10000 } },
	{ .num_addrs = 3, .addrs = { 400, 2500, 1000 } },
	{ .num_addrs = 3, .addrs = { 500, 3000, 4000 } },
	{ .num_addrs = 3, .addrs = { 600, 3500, 7000 } },
	{ .num_addrs = 3, .addrs = { 700, 4000, 10500 } },
	{ .num_addrs = 3, .addrs = { 800, 4500, 2000 } },
	{ .num_addrs = 3, .addrs = { 900, 5000, 5000 } },
	{ .num_addrs = 3, .addrs = { 1000, 5500, 8000 } },
	{ .num_addrs = 3, .addrs = { 1100, 6000, 10000 } },
	{ .num_addrs = 3, .addrs = { 1200, 6500, 1000 } },
	{ .num_addrs = 3, .addrs = { 1300, 7000, 4000 } },
	{ .num_addrs = 3, .addrs = { 1400, 7500, 7000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n16200_r1_3(void)
{
	return table_n16200_r1_3;
}
