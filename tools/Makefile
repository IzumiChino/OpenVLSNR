# SPDX-License-Identifier: GPL-2.0-or-later

CC		?= gcc
CFLAGS		?= -std=c11 -O2 -Wall -Wextra -Werror
CFLAGS		+= -I../include $(shell pkg-config --cflags libiio)
LDLIBS		= -L.. -ldvbs2x_vlsnr -lm $(shell pkg-config --libs libiio)

TOOLS		= openvlsnr-pluto-tx openvlsnr-pluto-rx

.PHONY: all clean

all: $(TOOLS)

../libdvbs2x_vlsnr.a:
	$(MAKE) -C .. libdvbs2x_vlsnr.a

openvlsnr-pluto-tx: openvlsnr-pluto-tx.c pluto.c pluto.h \
			../libdvbs2x_vlsnr.a
	$(CC) $(CFLAGS) -o $@ openvlsnr-pluto-tx.c pluto.c $(LDLIBS)

openvlsnr-pluto-rx: openvlsnr-pluto-rx.c pluto.c pluto.h \
			../libdvbs2x_vlsnr.a
	$(CC) $(CFLAGS) -o $@ openvlsnr-pluto-rx.c pluto.c $(LDLIBS)

clean:
	rm -f $(TOOLS)
