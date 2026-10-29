CC      ?= gcc
# -march=native: tuned for this machine. For a binary that runs on other
# x86-64 machines use "make CFLAGS=-O2": the AVX2 / BMI2 code is built in
# anyway and chosen at run time.
CFLAGS  ?= -O2 -march=native
ALL_CFLAGS = $(CFLAGS) -Wall -Wextra -std=gnu11 $(shell pkg-config --cflags libusb-1.0)
LDLIBS  += $(shell pkg-config --libs libusb-1.0) -lpthread -lm

# Windows (MSYS2 UCRT64 shell): sockets, and a self-contained .exe
ifeq ($(OS),Windows_NT)
LDLIBS  += -lws2_32
LDFLAGS += -static
endif

ch32trace: ch32trace.c vth_lut.h
	$(CC) $(ALL_CFLAGS) $(LDFLAGS) -o $@ ch32trace.c $(LDLIBS)

test: ch32trace
	./test.sh

bench: ch32trace
	./bench.sh

clean:
	rm -f ch32trace ch32trace.exe

.PHONY: test bench clean
