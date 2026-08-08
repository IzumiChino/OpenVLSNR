// SPDX-License-Identifier: GPL-2.0-or-later
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pluto.h"

#define DEFAULT_URI		"ip:192.168.2.1"
#define DEFAULT_SYMBOL_RATE	1250000
#define DEFAULT_SPS		2
#define DEFAULT_GAIN		40.0
#define PLUTO_BUFFER_SAMPLES	16384
#define RX_PACKET_CAPACITY	16

struct rx_options {
	const char	*uri;
	const char	*path;
	long long	frequency;
	long long	symbol_rate;
	unsigned int	sps;
	double		gain;
};

static volatile sig_atomic_t stop_requested;

static void stop_handler(int signal)
{
	(void)signal;
	stop_requested = 1;
}

static void usage(const char *name)
{
	fprintf(stderr,
		"Usage: %s -f HZ [options] FILE.ts\n"
		"  -u, --uri URI          IIO URI (default %s)\n"
		"  -f, --frequency HZ     RF center frequency\n"
		"  -r, --symbol-rate HZ   symbol rate (default %d)\n"
		"  -s, --sps N            samples per symbol (default %d)\n"
		"  -g, --gain DB          manual RX gain (default %.1f)\n",
		name, DEFAULT_URI, DEFAULT_SYMBOL_RATE, DEFAULT_SPS,
		DEFAULT_GAIN);
}

static int parse_longlong(const char *value, long long *result)
{
	char *end;
	long long parsed;

	errno = 0;
	parsed = strtoll(value, &end, 10);
	if (errno || *end || parsed <= 0)
		return -1;
	*result = parsed;
	return 0;
}

static int parse_uint(const char *value, unsigned int *result)
{
	long long parsed;

	if (parse_longlong(value, &parsed) < 0 || parsed > UINT_MAX)
		return -1;
	*result = (unsigned int)parsed;
	return 0;
}

static int parse_double(const char *value, double *result)
{
	char *end;
	double parsed;

	errno = 0;
	parsed = strtod(value, &end);
	if (errno || *end)
		return -1;
	*result = parsed;
	return 0;
}

static int parse_options(int argc, char **argv, struct rx_options *options)
{
	static const struct option long_options[] = {
		{ "uri", required_argument, NULL, 'u' },
		{ "frequency", required_argument, NULL, 'f' },
		{ "symbol-rate", required_argument, NULL, 'r' },
		{ "sps", required_argument, NULL, 's' },
		{ "gain", required_argument, NULL, 'g' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int option;

	memset(options, 0, sizeof(*options));
	options->uri = DEFAULT_URI;
	options->symbol_rate = DEFAULT_SYMBOL_RATE;
	options->sps = DEFAULT_SPS;
	options->gain = DEFAULT_GAIN;
	while ((option = getopt_long(argc, argv, "u:f:r:s:g:h",
				      long_options, NULL)) != -1) {
		switch (option) {
		case 'u':
			options->uri = optarg;
			break;
		case 'f':
			if (parse_longlong(optarg, &options->frequency) < 0)
				return -1;
			break;
		case 'r':
			if (parse_longlong(optarg, &options->symbol_rate) < 0)
				return -1;
			break;
		case 's':
			if (parse_uint(optarg, &options->sps) < 0)
				return -1;
			break;
		case 'g':
			if (parse_double(optarg, &options->gain) < 0)
				return -1;
			break;
		case 'h':
			usage(argv[0]);
			exit(0);
		default:
			return -1;
		}
	}
	if (!options->frequency || optind + 1 != argc ||
	    options->sps < 2 || options->sps > 16 ||
	    options->symbol_rate > LLONG_MAX / options->sps ||
	    options->gain < 0.0 || options->gain > 73.0)
		return -1;
	options->path = argv[optind];
	return 0;
}

static int write_packets(FILE *output, const uint8_t *packets,
			 unsigned int packet_count)
{
	if (!packet_count)
		return 0;
	if (fwrite(packets, DVBS2X_TS_PACKET_SIZE, packet_count, output) !=
	    packet_count) {
		perror("TS output");
		return -1;
	}
	return 0;
}

static int process_bbframe(struct dvbs2x_ts_rx *ts, const uint8_t *bbframe,
			   uint8_t *packets, FILE *output,
			   unsigned long long *packet_total)
{
	unsigned int packet_count;
	int ret;

	ret = dvbs2x_ts_rx_push(ts, bbframe, packets, RX_PACKET_CAPACITY,
				&packet_count);
	if (ret < 0)
		return ret;
	if (write_packets(output, packets, packet_count) < 0)
		return -1;
	*packet_total += packet_count;
	return 0;
}

int main(int argc, char **argv)
{
	struct rx_options options;
	struct pluto_config config;
	struct pluto_stream stream = { 0 };
	struct dvbs2x_demodulator demod = { 0 };
	struct dvbs2x_ts_rx ts = { 0 };
	struct dvbs2x_complex *samples = NULL;
	uint8_t packets[RX_PACKET_CAPACITY * DVBS2X_TS_PACKET_SIZE];
	uint8_t *bbframe = NULL;
	unsigned long long frame_count = 0;
	unsigned long long packet_count = 0;
	FILE *output = NULL;
	int ret = 1;

	if (parse_options(argc, argv, &options) < 0) {
		usage(argv[0]);
		return 2;
	}
	output = strcmp(options.path, "-") ? fopen(options.path, "wb") : stdout;
	if (!output) {
		perror(options.path);
		return 1;
	}
	dvbs2x_library_init();
	if (dvbs2x_demodulator_init(&demod, 0.35, options.sps, 0) < 0)
		goto out;
	bbframe = malloc(DVBS2X_LDPC_NORMAL);
	samples = malloc(PLUTO_BUFFER_SAMPLES * sizeof(*samples));
	if (!bbframe || !samples)
		goto out;
	config.uri = options.uri;
	config.frequency = options.frequency;
	config.sample_rate = options.symbol_rate * options.sps;
	config.bandwidth = options.symbol_rate * 135 / 100;
	config.gain = options.gain;
	if (pluto_rx_open(&stream, &config, PLUTO_BUFFER_SAMPLES) < 0)
		goto out;
	if (signal(SIGINT, stop_handler) == SIG_ERR ||
	    signal(SIGTERM, stop_handler) == SIG_ERR)
		goto out;
	fprintf(stderr, "RX %.3f MHz, %.3f Msym/s, %lld MS/s\n",
		(double)options.frequency / 1e6,
		(double)options.symbol_rate / 1e6, config.sample_rate / 1000000);
	while (!stop_requested) {
		unsigned int sample_len;
		unsigned int consumed;
		unsigned int frame_len;
		int dret;

		if (pluto_rx_read(&stream, samples, PLUTO_BUFFER_SAMPLES,
				  &sample_len) < 0)
			goto out;
		dret = dvbs2x_demodulate_bbframe_stream_ex(
			&demod, samples, sample_len, bbframe,
			DVBS2X_LDPC_NORMAL, &frame_len, &consumed);
		while (dret != DVBS2X_ERR_SHORT) {
			if (dret == 0) {
				if (!demod.modcod ||
				    (!ts.bb.k_bch &&
				     dvbs2x_ts_rx_init(&ts, demod.modcod) < 0))
					goto out;
				if (ts.bb.k_bch != demod.modcod->k_bch) {
					dvbs2x_ts_rx_reset(&ts);
					if (dvbs2x_ts_rx_init(&ts, demod.modcod) < 0)
						goto out;
				}
				dret = process_bbframe(&ts, bbframe, packets, output,
							&packet_count);
				if (dret == 0)
					frame_count++;
				else
					dvbs2x_ts_rx_reset(&ts);
			}
			frame_len = 0;
			consumed = 0;
			dret = dvbs2x_demodulate_bbframe_stream_ex(
				&demod, NULL, 0, bbframe, DVBS2X_LDPC_NORMAL,
				&frame_len, &consumed);
		}
	}
	if (ts.bb.k_bch) {
		unsigned int final_count;

		if (dvbs2x_ts_rx_finalize_unchecked(
			    &ts, packets, 1, &final_count) < 0 ||
		    write_packets(output, packets, final_count) < 0)
			goto out;
		packet_count += final_count;
	}
	fprintf(stderr, "received %llu TS packets from %llu PL frames\n",
		packet_count, frame_count);
	ret = 0;
out:
	pluto_stream_close(&stream);
	dvbs2x_demodulator_destroy(&demod);
	free(samples);
	free(bbframe);
	if (output && output != stdout)
		fclose(output);
	return ret;
}
