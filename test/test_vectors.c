/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Golden Table and Sequence Tests
 *
 * Integer fingerprints are derived from ETSI EN 302 307-1 V1.4.1 and
 * EN 302 307-2 V1.3.1 tables, cross-checked against GNU Radio gr-dtv
 * dvb_defines.h, dvb_bbscrambler_bb_impl.cc, and dvbs2_physical_cc_impl.cc.
 * No GNU Radio source or GPL-3.0-only table data is copied here.
 */

#include <stdint.h>
#include <stdio.h>

#include "dvbs2x_vlsnr.h"

#define FNV_OFFSET	UINT64_C(1469598103934665603)
#define FNV_PRIME	UINT64_C(1099511628211)

#define GOLDEN_WH	UINT64_C(0x4a2c61194fa55983)
#define GOLDEN_MODCOD	UINT64_C(0x42e7a8460aadf336)
#define GOLDEN_LDPC	UINT64_C(0x27a0c9fd353fddb0)
#define GOLDEN_SCRAMBLING UINT64_C(0x69124a8e4831960c)
#define GOLDEN_PLS	UINT64_C(0xd34486a1cfa3dafb)
#define GOLDEN_FEC	UINT64_C(0x6cc5fe79505b1465)
#define GOLDEN_LAYOUT	UINT64_C(0xe3c2f46c4a15e7bc)

static uint64_t hash_byte(uint64_t hash, uint8_t value)
{
	return (hash ^ value) * FNV_PRIME;
}

static uint64_t hash_u32(uint64_t hash, unsigned int value)
{
	unsigned int i;

	for (i = 0; i < 4; i++)
		hash = hash_byte(hash, (value >> (i * 8)) & 0xff);
	return hash;
}

static uint64_t hash_wh(void)
{
	int8_t seq[DVBS2X_VLSNR_WH_LEN];
	uint64_t hash = FNV_OFFSET;
	unsigned int index, i;

	for (index = 0; index < 16; index++) {
		dvbs2x_wh_generate(index, seq);
		for (i = 0; i < DVBS2X_VLSNR_WH_LEN; i++)
			hash = hash_byte(hash, (uint8_t)seq[i]);
	}
	return hash;
}

static uint64_t hash_modcods(void)
{
	uint64_t hash = FNV_OFFSET;
	unsigned int index;

	for (index = 1; index <= DVBS2X_VLSNR_NUM_MODCODS; index++) {
		const struct dvbs2x_modcod *mc =
			dvbs2x_vlsnr_get_modcod(index);
		const unsigned int fields[] = {
			mc->index, mc->modulation, mc->frame_type, mc->fec_len,
			mc->code_rate_num, mc->code_rate_den, mc->has_spread,
			mc->set, mc->pls_code, mc->k_bch, mc->n_bch,
			mc->bch_t, mc->k_ldpc, mc->q_ldpc, mc->xs, mc->xp,
			mc->p_period, mc->nbch_eff,
		};
		unsigned int i;

		for (i = 0; i < sizeof(fields) / sizeof(fields[0]); i++)
			hash = hash_u32(hash, fields[i]);
	}
	return hash;
}

static uint64_t hash_ldpc_tables(void)
{
	static const unsigned int indices[] = { 1, 2, 3, 4, 5, 6, 8, 9 };
	uint64_t hash = FNV_OFFSET;
	unsigned int n;

	for (n = 0; n < sizeof(indices) / sizeof(indices[0]); n++) {
		const struct dvbs2x_modcod *mc =
			dvbs2x_vlsnr_get_modcod(indices[n]);
		const struct dvbs2x_ldpc_table_entry *table;
		unsigned int groups, group, i;

		table = dvbs2x_ldpc_get_table(mc, &groups);
		hash = hash_u32(hash, indices[n]);
		hash = hash_u32(hash, groups);
		for (group = 0; group < groups; group++) {
			hash = hash_u32(hash, table[group].num_addrs);
			for (i = 0; i < table[group].num_addrs; i++)
				hash = hash_u32(hash, table[group].addrs[i]);
		}
	}
	return hash;
}

static uint64_t hash_pl_scrambling(void)
{
	struct dvbs2x_scrambler scrambler;
	uint64_t hash = FNV_OFFSET;
	unsigned int i;

	dvbs2x_scrambler_init(&scrambler, 0);
	for (i = 0; i < 4096; i++)
		hash = hash_byte(hash, dvbs2x_scrambler_next_rn(&scrambler));
	return hash;
}

static uint64_t hash_pls(void)
{
	struct dvbs2x_complex symbols[DVBS2X_PLHEADER_LEN];
	static const unsigned int codes[] = {
		DVBS2X_PLS_VLSNR_SET1, DVBS2X_PLS_VLSNR_SET2,
	};
	uint64_t hash = FNV_OFFSET;
	unsigned int code, i;

	for (code = 0; code < sizeof(codes) / sizeof(codes[0]); code++) {
		dvbs2x_plheader_generate(codes[code], symbols);
		for (i = 0; i < DVBS2X_PLHEADER_LEN; i++) {
			uint8_t quadrant = symbols[i].i < 0.0;

			quadrant |= (symbols[i].q < 0.0) << 1;
			hash = hash_byte(hash, quadrant);
		}
	}
	return hash;
}

static uint64_t hash_fec_positions(void)
{
	uint64_t hash = FNV_OFFSET;
	unsigned int index;

	for (index = 1; index <= DVBS2X_VLSNR_NUM_MODCODS; index++) {
		const struct dvbs2x_modcod *mc =
			dvbs2x_vlsnr_get_modcod(index);
		unsigned int i, p_idx = 0;

		hash = hash_u32(hash, index);
		for (i = mc->xs; i < mc->k_ldpc; i++)
			hash = hash_u32(hash, i);
		for (i = 0; i < mc->fec_len - mc->k_ldpc; i++) {
			if (mc->xp && p_idx < mc->xp &&
			    i == p_idx * mc->p_period) {
				p_idx++;
				continue;
			}
			hash = hash_u32(hash, mc->k_ldpc + i);
		}
	}
	return hash;
}

static uint64_t hash_layouts(void)
{
	uint64_t hash = FNV_OFFSET;
	unsigned int index;

	for (index = 1; index <= DVBS2X_VLSNR_NUM_MODCODS; index++) {
		const struct dvbs2x_modcod *mc =
			dvbs2x_vlsnr_get_modcod(index);
		struct dvbs2x_vlsnr_layout layout = { 0 };
		unsigned int i;

		if (dvbs2x_vlsnr_build_layout(mc, &layout) < 0)
			return 0;
		hash = hash_u32(hash, layout.num_data);
		hash = hash_u32(hash, layout.num_pilot);
		hash = hash_u32(hash, layout.field_len);
		for (i = 0; i < layout.field_len; i++)
			hash = hash_byte(hash, layout.is_pilot[i]);
		dvbs2x_vlsnr_free_layout(&layout);
	}
	return hash;
}

static int test_header_scale(void)
{
	struct dvbs2x_complex header[DVBS2X_VLSNR_HDR_LEN];
	const struct dvbs2x_modcod *mc;
	unsigned int offset = UINT32_MAX;
	unsigned int modcod = 0;
	unsigned int i;
	double confidence;

	mc = dvbs2x_vlsnr_get_modcod(9);
	if (!mc)
		return -1;
	dvbs2x_vlsnr_header_generate(mc, header);
	for (i = 0; i < DVBS2X_VLSNR_HDR_LEN; i++) {
		header[i].i *= 0.001;
		header[i].q *= 0.001;
	}
	confidence = dvbs2x_vlsnr_header_sync(header + 2,
		DVBS2X_VLSNR_WH_LEN, 128, &offset, &modcod);
	if (confidence < 0.99 || offset || modcod != mc->index)
		return -1;
	return 0;
}

int main(void)
{
	unsigned int index;

	printf("DVB-S2X VL-SNR Golden Vector Tests\n");
	printf("==================================\n");
	if (hash_wh() != GOLDEN_WH || hash_modcods() != GOLDEN_MODCOD ||
	    hash_ldpc_tables() != GOLDEN_LDPC ||
	    hash_pl_scrambling() != GOLDEN_SCRAMBLING ||
	    hash_pls() != GOLDEN_PLS || hash_fec_positions() != GOLDEN_FEC ||
	    hash_layouts() != GOLDEN_LAYOUT || test_header_scale() < 0) {
		printf("FAIL: golden fingerprint mismatch\n");
		return 1;
	}
	for (index = 1; index <= DVBS2X_VLSNR_NUM_MODCODS; index++) {
		const struct dvbs2x_modcod *mc =
			dvbs2x_vlsnr_get_modcod(index);
		struct dvbs2x_vlsnr_layout layout = { 0 };
		unsigned int frame_len;

		if (dvbs2x_vlsnr_build_layout(mc, &layout) < 0)
			return 1;
		frame_len = DVBS2X_PLHEADER_LEN + DVBS2X_VLSNR_HDR_LEN +
			layout.field_len;
		dvbs2x_vlsnr_free_layout(&layout);
		if (frame_len != (index <= 6 ?
				  DVBS2X_VLSNR_FRAME_LONG :
				  DVBS2X_VLSNR_FRAME_SHORT)) {
			printf("FAIL: MODCOD %u frame length\n", index);
			return 1;
		}
	}
	printf("PASS\n");
	return 0;
}
