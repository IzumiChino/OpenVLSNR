// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * DVB-S2X LDPC Parity Address Tables
 *
 * Placeholder tables for VL-SNR LDPC codes.
 * These tables define the positions of 1s in the first column
 * of each circulant group in the parity check matrix.
 *
 * The actual values must be populated from ETSI EN 302 307-2
 * Annex B tables for each code rate and frame length.
 *
 * TODO: Fill in actual table values from the standard.
 */

#include "ldpc.h"

/*
 * Rate 2/9, Normal frame (64800 bits)
 * K = 14400, M = 50400, Q = 140
 * Number of groups = 14400 / 360 = 40
 */
static const struct dvbs2x_ldpc_table_entry table_n64800_r2_9[] = {
	/* Group 0 */
	{ .num_addrs = 13, .addrs = {
		0, 2084, 1613, 1548, 1286, 1460, 3196, 4297,
		2481, 3369, 3451, 4620, 2622 } },
	/* Group 1 */
	{ .num_addrs = 13, .addrs = {
		1, 2039, 1605, 1537, 1280, 1458, 3190, 4290,
		2475, 3363, 3445, 4614, 2616 } },
	/* Groups 2-39: placeholder with minimal connectivity */
	{ .num_addrs = 3, .addrs = { 10, 200, 5000 } },
	{ .num_addrs = 3, .addrs = { 20, 400, 10000 } },
	{ .num_addrs = 3, .addrs = { 30, 600, 15000 } },
	{ .num_addrs = 3, .addrs = { 40, 800, 20000 } },
	{ .num_addrs = 3, .addrs = { 50, 1000, 25000 } },
	{ .num_addrs = 3, .addrs = { 60, 1200, 30000 } },
	{ .num_addrs = 3, .addrs = { 70, 1400, 35000 } },
	{ .num_addrs = 3, .addrs = { 80, 1600, 40000 } },
	{ .num_addrs = 3, .addrs = { 90, 1800, 45000 } },
	{ .num_addrs = 3, .addrs = { 100, 2000, 50000 } },
	{ .num_addrs = 3, .addrs = { 110, 2200, 4000 } },
	{ .num_addrs = 3, .addrs = { 120, 2400, 8000 } },
	{ .num_addrs = 3, .addrs = { 130, 2600, 12000 } },
	{ .num_addrs = 3, .addrs = { 140, 2800, 16000 } },
	{ .num_addrs = 3, .addrs = { 150, 3000, 20000 } },
	{ .num_addrs = 3, .addrs = { 160, 3200, 24000 } },
	{ .num_addrs = 3, .addrs = { 170, 3400, 28000 } },
	{ .num_addrs = 3, .addrs = { 180, 3600, 32000 } },
	{ .num_addrs = 3, .addrs = { 190, 3800, 36000 } },
	{ .num_addrs = 3, .addrs = { 200, 4000, 40000 } },
	{ .num_addrs = 3, .addrs = { 210, 4200, 44000 } },
	{ .num_addrs = 3, .addrs = { 220, 4400, 48000 } },
	{ .num_addrs = 3, .addrs = { 230, 4600, 2000 } },
	{ .num_addrs = 3, .addrs = { 240, 4800, 6000 } },
	{ .num_addrs = 3, .addrs = { 250, 5000, 10000 } },
	{ .num_addrs = 3, .addrs = { 260, 5200, 14000 } },
	{ .num_addrs = 3, .addrs = { 270, 5400, 18000 } },
	{ .num_addrs = 3, .addrs = { 280, 5600, 22000 } },
	{ .num_addrs = 3, .addrs = { 290, 5800, 26000 } },
	{ .num_addrs = 3, .addrs = { 300, 6000, 30000 } },
	{ .num_addrs = 3, .addrs = { 310, 6200, 34000 } },
	{ .num_addrs = 3, .addrs = { 320, 6400, 38000 } },
	{ .num_addrs = 3, .addrs = { 330, 6600, 42000 } },
	{ .num_addrs = 3, .addrs = { 340, 6800, 46000 } },
	{ .num_addrs = 3, .addrs = { 350, 7000, 50000 } },
	{ .num_addrs = 3, .addrs = { 360, 7200, 3000 } },
	{ .num_addrs = 3, .addrs = { 370, 7400, 7000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n64800_r2_9(void)
{
	return table_n64800_r2_9;
}

/*
 * Rate 1/5, Medium frame (32400 bits)
 * K = 6480, M = 25920, Q = 72
 * Number of groups = 6480 / 360 = 18
 */
static const struct dvbs2x_ldpc_table_entry table_n32400_r1_5[18] = {
	{ .num_addrs = 3, .addrs = { 0, 1000, 5000 } },
	{ .num_addrs = 3, .addrs = { 100, 2000, 10000 } },
	{ .num_addrs = 3, .addrs = { 200, 3000, 15000 } },
	{ .num_addrs = 3, .addrs = { 300, 4000, 20000 } },
	{ .num_addrs = 3, .addrs = { 400, 5000, 25000 } },
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
	{ .num_addrs = 3, .addrs = { 1700, 18000, 25000 } },
};

const struct dvbs2x_ldpc_table_entry *dvbs2x_ldpc_get_table_n32400_r1_5(void)
{
	return table_n32400_r1_5;
}
