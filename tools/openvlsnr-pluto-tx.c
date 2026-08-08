// SPDX-License-Identifier: GPL-2.0-or-later
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pluto.h"

#define DEFAULT_URI		"ip:192.168.2.1"
#define DEFAULT_SYMBOL_RATE	1250000
#define DEFAULT_SPS		2
#define DEFAULT_MODCOD		9
#define DEFAULT_GAIN		(-30.0)
#define DEFAULT_SCALE		0.5
#define PLUTO_BUFFER_SAMPLES	16384

struct tx_options {
	const char	*uri;
	const char	*path;
	long long	frequency;
	long long	symbol_rate;
	unsigned int	sps;
	unsigned int	modcod;
	double		gain;
	double		scale;
	int		repeat;
};

static void usage(const char *name)
{
	fprintf(stderr,
		"Usage: %s -f HZ [options] FILE.ts\n"
		"  -u, --uri URI          IIO URI (default %s)\n"
		"  -f, --frequency HZ     RF center frequency\n"
		"  -r, --symbol-rate HZ   symbol rate (default %d)\n"
		"  -s, --sps N            samples per symbol (default %d)\n"
		"  -m, --modcod N         VL-SNR MODCOD 1-9 (default %d)\n"
		"  -g, --gain DB          TX hardware gain (default %.1f)\n"
		"  -a, --amplitude A      digital amplitude 0 < A <= 1\n"
		"  -R, --repeat           repeat the TS file continuously\n",
		name, DEFAULT_URI, DEFAULT_SYMBOL_RATE, DEFAULT_SPS,
		DEFAULT_MODCOD, DEFAULT_GAIN);
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

static int parse_options(int argc, char **argv, struct tx_options *options)
{
	static const struct option long_options[] = {
		{ "uri", required_argument, NULL, 'u' },
		{ "frequency", required_argument, NULL, 'f' },
		{ "symbol-rate", required_argument, NULL, 'r' },
		{ "sps", required_argument, NULL, 's' },
		{ "modcod", required_argument, NULL, 'm' },
		{ "gain", required_argument, NULL, 'g' },
		{ "amplitude", required_argument, NULL, 'a' },
		{ "repeat", no_argument, NULL, 'R' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int option;

	memset(options, 0, sizeof(*options));
	options->uri = DEFAULT_URI;
	options->symbol_rate = DEFAULT_SYMBOL_RATE;
	options->sps = DEFAULT_SPS;
	options->modcod = DEFAULT_MODCOD;
	options->gain = DEFAULT_GAIN;
	options->scale = DEFAULT_SCALE;
	while ((option = getopt_long(argc, argv, "u:f:r:s:m:g:a:Rh",
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
		case 'm':
			if (parse_uint(optarg, &options->modcod) < 0)
				return -1;
			break;
		case 'g':
			if (parse_double(optarg, &options->gain) < 0)
				return -1;
			break;
		case 'a':
			if (parse_double(optarg, &options->scale) < 0)
				return -1;
			break;
		case 'R':
			options->repeat = 1;
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
	    options->modcod < 1 || options->modcod > 9 ||
	    options->gain < -89.75 || options->gain > 0.0 ||
	    options->scale <= 0.0 || options->scale > 1.0)
		return -1;
	options->path = argv[optind];
	if (options->repeat && !strcmp(options->path, "-"))
		return -1;
	return 0;
}

static void make_null_packet(uint8_t *packet)
{
	memset(packet, 0xff, DVBS2X_TS_PACKET_SIZE);
	packet[0] = 0x47;
	packet[1] = 0x1f;
	packet[2] = 0xff;
	packet[3] = 0x10;
}

static int send_frame(struct dvbs2x_modulator *mod,
		      struct pluto_stream *stream, const uint8_t *bbframe,
		      struct dvbs2x_complex *symbols,
		      struct dvbs2x_complex *samples, double scale)
{
	unsigned int symbol_len;
	unsigned int sample_len;

	if (dvbs2x_modulate_bbframe_symbols_ex(
		    mod, bbframe, symbols, DVBS2X_VLSNR_FRAME_LONG,
		    &symbol_len) < 0)
		return -1;
	dvbs2x_rrc_upsample(&mod->tx_filter, symbols, symbol_len, samples,
			    &sample_len);
	return pluto_tx_write(stream, samples, sample_len, scale);
}

static void report_progress(unsigned long long frame_count,
			    long long sample_rate)
{
	if (frame_count == 1)
		fprintf(stderr, "streaming started at %.3f MS/s\n",
			(double)sample_rate / 1e6);
	else if (!(frame_count % 100))
		fprintf(stderr, "sent %llu PL frames\r", frame_count);
}

int main(int argc, char **argv)
{
	struct tx_options options;
	struct pluto_config config;
	struct pluto_stream stream = { 0 };
	struct dvbs2x_modulator mod = { 0 };
	struct dvbs2x_ts_tx ts = { 0 };
	const struct dvbs2x_modcod *mc;
	struct dvbs2x_complex *symbols = NULL;
	struct dvbs2x_complex *samples = NULL;
	uint8_t packet[DVBS2X_TS_PACKET_SIZE];
	uint8_t *bbframe = NULL;
	unsigned int frame_len = 0;
	unsigned long long packet_count = 0;
	unsigned long long frame_count = 0;
	unsigned long long pass_packets = 0;
	FILE *input = NULL;
	int ret = 1;

	if (parse_options(argc, argv, &options) < 0) {
		usage(argv[0]);
		return 2;
	}
	input = strcmp(options.path, "-") ? fopen(options.path, "rb") : stdin;
	if (!input) {
		perror(options.path);
		return 1;
	}
	dvbs2x_library_init();
	mc = dvbs2x_vlsnr_get_modcod(options.modcod);
	if (!mc || dvbs2x_modulator_init(&mod, options.modcod, 0.35,
					 options.sps, 0) < 0 ||
	    dvbs2x_ts_tx_init(&ts, mc, DVBS2X_RO_0_35) < 0)
		goto out;
	bbframe = malloc(mc->k_bch);
	symbols = malloc(DVBS2X_VLSNR_FRAME_LONG * sizeof(*symbols));
	samples = malloc(DVBS2X_VLSNR_FRAME_LONG * options.sps *
			 sizeof(*samples));
	if (!bbframe || !symbols || !samples)
		goto out;
	config.uri = options.uri;
	config.frequency = options.frequency;
	config.sample_rate = options.symbol_rate * options.sps;
	config.bandwidth = options.symbol_rate * 135 / 100;
	config.gain = options.gain;
	if (pluto_tx_open(&stream, &config, PLUTO_BUFFER_SAMPLES) < 0)
		goto out;
	fprintf(stderr, "TX %.3f MHz, %.3f Msym/s, %.3f MS/s, MODCOD %u\n",
		(double)options.frequency / 1e6,
		(double)options.symbol_rate / 1e6,
		(double)config.sample_rate / 1e6,
		options.modcod);
	for (;;) {
		if (fread(packet, sizeof(packet), 1, input) != 1) {
			if (ferror(input)) {
				perror("TS input");
				goto out;
			}
			if (!options.repeat)
				break;
			if (!pass_packets || fseek(input, 0, SEEK_SET) < 0) {
				fprintf(stderr, "cannot repeat TS input\n");
				goto out;
			}
			clearerr(input);
			pass_packets = 0;
			continue;
		}
		if (packet[0] != 0x47) {
			fprintf(stderr, "invalid TS sync at packet %llu\n",
				packet_count);
			goto out;
		}
		if (dvbs2x_ts_tx_push(&ts, packet, bbframe, mc->k_bch,
				      &frame_len) < 0)
			goto out;
		packet_count++;
		pass_packets++;
		if (!frame_len)
			continue;
		if (send_frame(&mod, &stream, bbframe, symbols, samples,
			       options.scale) < 0)
			goto out;
		frame_count++;
		report_progress(frame_count, config.sample_rate);
	}
	if (ts.data_len) {
		make_null_packet(packet);
		while (!frame_len) {
			if (dvbs2x_ts_tx_push(&ts, packet, bbframe, mc->k_bch,
					      &frame_len) < 0)
				goto out;
		}
		if (send_frame(&mod, &stream, bbframe, symbols, samples,
			       options.scale) < 0)
			goto out;
		frame_count++;
		report_progress(frame_count, config.sample_rate);
	}
	if (pluto_tx_flush(&stream) < 0)
		goto out;
	fprintf(stderr, "\nsent %llu TS packets in %llu PL frames\n",
		packet_count, frame_count);
	ret = 0;
out:
	pluto_stream_close(&stream);
	dvbs2x_ts_tx_destroy(&ts);
	dvbs2x_modulator_destroy(&mod);
	free(samples);
	free(symbols);
	free(bbframe);
	if (input && input != stdin)
		fclose(input);
	return ret;
}
