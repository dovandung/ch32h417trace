#!/bin/sh
# Cross-build ch32trace.exe for 64-bit Windows on a Linux PC: one static
# .exe, no DLLs needed (Windows 10 or later, for the UCRT runtime).
#
# It downloads the llvm-mingw toolchain and libusb 1.0.30 (>= 1.0.30 is
# needed for WinUSB RAW_IO, which keeps several USB reads in flight; without
# it WinUSB cannot keep up with 300 MB/s) into ./winbuild.
set -e
LLVM_MINGW=20261020
LIBUSB=1.0.30
W=$(pwd)/winbuild
mkdir -p "$W"
cd "$W"

if [ ! -d llvm-mingw ]; then
	curl -fL -o lm.tar.xz "https://github.com/mstorsjo/llvm-mingw/releases/download/$LLVM_MINGW/llvm-mingw-$LLVM_MINGW-ucrt-ubuntu-22.04-x86_64.tar.xz"
	tar xJf lm.tar.xz
	mv "llvm-mingw-$LLVM_MINGW-ucrt-ubuntu-22.04-x86_64" llvm-mingw
fi
export PATH="$W/llvm-mingw/bin:$PATH"

if [ ! -f usb/lib/libusb-1.0.a ]; then
	curl -fL -o libusb.tar.bz2 "https://github.com/libusb/libusb/releases/download/v$LIBUSB/libusb-$LIBUSB.tar.bz2"
	tar xjf libusb.tar.bz2
	(cd "libusb-$LIBUSB" &&
	 ./configure --host=x86_64-w64-mingw32 --prefix="$W/usb" \
		--enable-static --disable-shared > /dev/null &&
	 make -j4 > /dev/null && make install > /dev/null)
fi

cd ..
x86_64-w64-mingw32-clang -O2 -Wall -Wextra -std=gnu11 -static \
	-I"$W/usb/include/libusb-1.0" -o ch32trace.exe ch32trace.c \
	"$W/usb/lib/libusb-1.0.a" -lws2_32 -lpthread
echo "built ch32trace.exe"
