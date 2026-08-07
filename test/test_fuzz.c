/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DVB-S2X VL-SNR Robustness Test
 *
 * Feeds random IQ data to the demodulator to verify it never
 * crashes, only returns error codes gracefully.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dvbs2x_vlsnr.h"

#define FUZZ_ITERATIONS	100
#define MAX_BUF_LEN	80000

int main(int argc, char *argv[])
{
	struct dvbs2x_demodulator demod;
	struct dvbs2x_complex *buf;
	uint8_t *out;
	unsigned int iterations = FUZZ_ITERATIONS;
	unsigned int seed = 1;
	unsigned int i, n;
	unsigned int nosync = 0;
	unsigned int other = 0;
	unsigned int ok = 0;
	int ret;

	if (argc >= 2)
		iterations = (unsigned int)atoi(argv[1]);
	if (argc >= 3)
		seed = (unsigned int)atoi(argv[2]);

	srand(seed);

	dvbs2x_library_init();
	if (dvbs2x_demodulator_init(&demod, 0.35, 2, 0) < 0)
		return 1;

	buf = malloc(MAX_BUF_LEN * sizeof(*buf));
	out = malloc(65536);
	if (!buf || !out) {
		free(buf);
		free(out);
		dvbs2x_demodulator_destroy(&demod);
		return 1;
	}

	printf("DVB-S2X VL-SNR Robustness Test\n");
	printf("==============================\n");
	printf("Feeding %u random buffers with seed %u...\n",
	       iterations, seed);

	for (i = 0; i < iterations; i++) {
		unsigned int len;

		len = (unsigned int)(rand() % (MAX_BUF_LEN - 100)) + 100;
		unsigned int rec_len = 0;

		/* Fill with random IQ */
		for (n = 0; n < len; n++) {
			buf[n].i = ((double)rand() / RAND_MAX) * 2.0 -
				   1.0;
			buf[n].q = ((double)rand() / RAND_MAX) * 2.0 -
				   1.0;
		}

		ret = dvbs2x_demodulate(&demod, buf, len, out, &rec_len);

		if (ret == DVBS2X_ERR_NOSYNC || ret == DVBS2X_ERR_SHORT)
			nosync++;
		else if (ret == DVBS2X_OK)
			ok++;
		else
			other++;
	}

	printf("  NOSYNC/SHORT: %u\n", nosync);
	printf("  Other error:  %u\n", other);
	printf("  Decoded(!):   %u\n", ok);
	printf("\nNo crashes.  PASS\n");

	free(buf);
	free(out);
	dvbs2x_demodulator_destroy(&demod);
	return 0;
}
