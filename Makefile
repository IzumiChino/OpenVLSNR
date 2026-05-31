# SPDX-License-Identifier: GPL-2.0-or-later
#
# DVB-S2X VL-SNR Modulator/Demodulator
#

CC		= gcc
CFLAGS		= -Wall -Wextra -Werror -std=c11 -O2
CFLAGS		+= -Iinclude
LDFLAGS		= -lm

# Install paths (override on the command line, e.g. make install PREFIX=/usr)
PREFIX		?= /usr/local
libdir		= $(PREFIX)/lib
includedir	= $(PREFIX)/include
pkgincludedir	= $(includedir)/dvbs2x
HEADERS		= $(wildcard include/*.h)

INSTALL		= install
INSTALL_DATA	= $(INSTALL) -m 644

# Debug build
ifdef DEBUG
CFLAGS		+= -g -O0 -DDEBUG
endif

# Source directories
SRCDIR		= src
TBLDIR		= tables
TESTDIR		= test

# Source files
SRCS		= $(wildcard $(SRCDIR)/*.c)
TBL_SRCS	= $(wildcard $(TBLDIR)/*.c)
TEST_SRCS	= $(wildcard $(TESTDIR)/*.c)

# Object files
OBJS		= $(SRCS:.c=.o)
TBL_OBJS	= $(TBL_SRCS:.c=.o)
ALL_OBJS	= $(OBJS) $(TBL_OBJS)

# Library
LIB		= libdvbs2x_vlsnr.a

# Test binaries
TESTS		= $(TEST_SRCS:$(TESTDIR)/%.c=$(TESTDIR)/%)

.PHONY: all clean test install uninstall

all: $(LIB)

$(LIB): $(ALL_OBJS)
	$(AR) rcs $@ $^

$(SRCDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TBLDIR)/%.o: $(TBLDIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

# Test targets
test: $(TESTS)
	@for t in $(TESTS); do \
		echo "Running $$t..."; \
		./$$t || exit 1; \
	done
	@echo "All tests passed."

$(TESTDIR)/%: $(TESTDIR)/%.c $(LIB)
	$(CC) $(CFLAGS) -o $@ $< -L. -ldvbs2x_vlsnr $(LDFLAGS)

clean:
	rm -f $(ALL_OBJS) $(LIB) $(TESTS)
	rm -f $(TESTDIR)/*.o

install: $(LIB)
	$(INSTALL) -d $(DESTDIR)$(libdir)
	$(INSTALL_DATA) $(LIB) $(DESTDIR)$(libdir)/
	$(INSTALL) -d $(DESTDIR)$(pkgincludedir)
	$(INSTALL_DATA) $(HEADERS) $(DESTDIR)$(pkgincludedir)/

uninstall:
	rm -f $(DESTDIR)$(libdir)/$(LIB)
	rm -rf $(DESTDIR)$(pkgincludedir)
