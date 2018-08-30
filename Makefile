all:
	@mkdir -p default/dep
	@avr-gcc -mmcu=atmega32u4 -gdwarf-2 -Os -fsigned-char -ffunction-sections -ffunction-sections -MD -MP -MT default/demo.o -MF default/dep/demo.o.d  -c demo.c -o default/demo.o
	@avr-gcc -mmcu=atmega32u4 -Wl,-Map=EVK527-ATMega32U4-usbdevice_cdc.map,--cref,--gc-sections,--relax default/demo.o -o fw.elf
	@avr-objcopy -O ihex fw.elf fw.hex

objdump:
	@avr-objdump -d fw.elf

flash:
	@avrdude -qq -c usbasp -p atmega32u4 -U flash:w:fw.hex

clean:
	@git clean -X -d -f -e '!*.eps'

.PHONY: $(wildcard *.eps)

cdc-structure.eps: cdc-structure.png
	@convert $< $@
	@imgsize $@ 7.5 -

stall-control-write-without-data-stage.eps: stall-control-write-without-data-stage.gif
	@convert $< $@
	@imgsize $@
