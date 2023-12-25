#!/usr/bin/env bash
set -ve

TARGET="atmega328p"
PORT="/dev/ttyUSB0"
PROGRAMMER="-P ${PORT} -b 19200 -c avrisp"

# This HFUSE value (0xde) uses the smallest bootloader (256 bytes) and places
# the reset vector within it.  The interpreter places the IVT there as well.
avrdude ${PROGRAMMER} -p ${TARGET} -U hfuse:w:0xde:m -U flash:w:$1

