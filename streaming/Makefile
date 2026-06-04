# SPDX-License-Identifier: GPL-2.0-or-later
#
# Makefile for DVB-S2X VL-SNR Network Streaming Layer
#
# Usage:
#   make              - Build the library
#   make clean        - Remove build artifacts
#   make install      - Install library and headers (DESTDIR/PREFIX aware)
#

CC		= gcc
AR		= ar
CFLAGS		= -Wall -Wextra -Werror -std=c11 -O2 -g
CFLAGS		+= -Iinclude -I../include
LDFLAGS		= -lm

# Platform detection
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	PLATFORM = linux
	LDFLAGS += -lpthread -lrt
else ifeq ($(OS),Windows_NT)
	PLATFORM = windows
	CFLAGS += -D_WIN32_WINNT=0x0601 -DWIN32_LEAN_AND_MEAN
	LDFLAGS += -lws2_32 -lwinmm
else
	PLATFORM = unknown
endif

# AddressSanitizer support (make ASAN=1)
ifeq ($(ASAN),1)
	CFLAGS += -fsanitize=address -fno-omit-frame-pointer
	LDFLAGS += -fsanitize=address
endif

# Source files
SRCS		= src/dvbs2x_platform.c \
		  src/crc32.c \
		  src/protocol.c \
		  src/tcp.c \
		  src/udp.c
OBJS		= $(SRCS:.c=.o)

# Library
LIB		= libdvbs2x_streaming.a

.PHONY: all clean install

all: $(LIB)

$(LIB): $(OBJS)
	$(AR) rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(LIB)

install: $(LIB)
	install -d $(DESTDIR)$(PREFIX)/lib
	install -m 644 $(LIB) $(DESTDIR)$(PREFIX)/lib/
	install -d $(DESTDIR)$(PREFIX)/include/dvbs2x
	install -m 644 include/*.h $(DESTDIR)$(PREFIX)/include/dvbs2x/

# Dependencies
src/dvbs2x_platform.o: src/dvbs2x_platform.c include/dvbs2x_platform.h
src/crc32.o: src/crc32.c include/dvbs2x_protocol.h
src/protocol.o: src/protocol.c include/dvbs2x_protocol.h
src/tcp.o: src/tcp.c include/dvbs2x_tcp.h include/dvbs2x_platform.h
src/udp.o: src/udp.c include/dvbs2x_udp.h include/dvbs2x_platform.h
