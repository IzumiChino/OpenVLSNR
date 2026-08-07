#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

set -eu

sanitizers=${1:-address,undefined}
src_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/openvlsnr-sanitize.XXXXXX")

cleanup()
{
	rm -rf -- "$build_dir"
}
trap cleanup EXIT HUP INT TERM

tar -C "$src_dir" \
	--exclude=.git \
	--exclude='*.o' \
	--exclude='test/test_*' \
	-cf - . | tar -C "$build_dir" -xf -

# Test sources share the test_ prefix, so copy them after excluding binaries.
mkdir -p "$build_dir/test"
for source in "$src_dir"/test/*.c; do
	cp "$source" "$build_dir/test/"
done
cp "$src_dir/test/run-sanitizers.sh" "$build_dir/test/"

san_cflags="-Wall -Wextra -Werror -std=c11 -O1 -g"
san_cflags="$san_cflags -fsanitize=$sanitizers -fno-omit-frame-pointer"
san_cflags="$san_cflags -Iinclude"

make -C "$build_dir" \
	CFLAGS="$san_cflags" \
	LDFLAGS="-lm -fsanitize=$sanitizers" \
	test
