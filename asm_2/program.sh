#!/usr/bin/env bash
set -ve

TARGET="atmega328p"
PORT="/dev/ttyUSB0"
PROGRAMMER="-P ${PORT} -b 19200 -c avrisp"

avrdude ${PROGRAMMER} -p ${TARGET} -U flash:w:siggen.hex

