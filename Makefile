all:
	mkdir -p default/dep
	avr-gcc -mmcu=atmega32u4 -Wall -gdwarf-2 -Os -fsigned-char -ffunction-sections -ffunction-sections -MD -MP -MT default/main.o -MF default/dep/main.o.d  -c main.c -o default/main.o
	avr-gcc -mmcu=atmega32u4 -Wl,-Map=EVK527-ATMega32U4-usbdevice_cdc.map,--cref,--gc-sections,--relax default/main.o -o EVK527-ATMega32U4-usbdevice_cdc.elf

flash:
	@avr-objcopy -O ihex EVK527-ATMega32U4-usbdevice_cdc.elf EVK527-ATMega32U4-usbdevice_cdc.hex
	avrdude -qq -c usbasp -p atmega32u4 -U flash:w:EVK527-ATMega32U4-usbdevice_cdc.hex

objdump: all
	avr-objdump -d EVK527-ATMega32U4-usbdevice_cdc.elf >x

clean:
	@git clean -X -d -f

.PHONY: $(wildcard *.eps)

cdc-structure.eps: cdc-structure.png
	@convert $< $@
	@imgsize $@ 7.5 -
