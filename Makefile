default: all

# Name: Makefile
# Author: Scott Perry <dev@numist.net>
# Copyright: 2015
# License: MIT

# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected. We recommend that you leave it undefined and
#                add settings like this to your ~/.avrduderc file:
#                   default_programmer = "dragon_isp";
#                   default_serial = "usb";
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     ?= attiny88

# 8Mhz
CLOCK      ?= 8000000

# For computing fuse byte values for other devices and options see
# the fuse bit calculator at http://www.engbedded.com/fusecalc/
FUSES      = -U lfuse:w:0xee:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m

AVRDUDE_PATH ?= avrdude
GCC_PATH ?= avr-gcc
PROGRAMMER ?= -c usbtiny

AVRDUDE = $(AVRDUDE_PATH) $(PROGRAMMER) -p $(DEVICE) -v

flash:	all
	$(AVRDUDE) -B 2 -U flash:w:out/attiny88_factory.hex:i

fuse:
	$(AVRDUDE) -B 100 $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: all fuse flash

clean:
	make -C firmware clean
	rm -fr out/*

all: build flashing-tool

build:
	make -C firmware
	mkdir -p out
	cp firmware/main.hex out/attiny88_keyscanner.hex
	./tools/make_factory_firmware.py

flashing-tool: build
	mkdir -p out/attiny_flasher
	cp etc/flasher_Makefile out/attiny_flasher/Makefile
	cp etc/flash_firmware.ino out/attiny_flasher/attiny_flasher.ino
	python2.7 ./tools/hex_to_atmega.py out/attiny88_keyscanner.hex > out/attiny_flasher/attiny_flasher.h
	mkdir -p out/dist
	cd out && tar czvf dist/attiny_flasher-`git describe`.tar.gz attiny_flasher


.PHONY: default all clean install flash fuse
