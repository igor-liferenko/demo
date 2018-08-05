all:
	mkdir -p default/dep
	avr-gcc -mmcu=atmega32u4 -Wall -gdwarf-2 -Os -fsigned-char -ffunction-sections -ffunction-sections -MD -MP -MT default/usb_specific_request.o -MF default/dep/usb_specific_request.o.d  -c usb_specific_request.c -o default/usb_specific_request.o
	avr-gcc -mmcu=atmega32u4 -Wall -gdwarf-2 -Os -fsigned-char -ffunction-sections -ffunction-sections -MD -MP -MT default/main.o -MF default/dep/main.o.d  -c main.c -o default/main.o
	avr-gcc -mmcu=atmega32u4 -x assembler-with-cpp -Wa,-gdwarf2 -c flash_drv.s -o default/flash_drv.o
	avr-gcc -mmcu=atmega32u4 -Wl,-Map=EVK527-ATMega32U4-usbdevice_cdc.map,--cref,--gc-sections,--relax default/usb_specific_request.o default/main.o default/flash_drv.o    -o EVK527-ATMega32U4-usbdevice_cdc.elf

flash:
	@avr-objcopy -O ihex EVK527-ATMega32U4-usbdevice_cdc.elf EVK527-ATMega32U4-usbdevice_cdc.hex
	avrdude -qq -c usbasp -p atmega32u4 -U flash:w:EVK527-ATMega32U4-usbdevice_cdc.hex

indent:
	git diff --exit-code HEAD
	for i in *.c; do indent -kr -i2 -ci2 -lp -ss --no-tabs -nce $$i; done; rm *.c~

objdump: all
	avr-objdump -d EVK527-ATMega32U4-usbdevice_cdc.elf >x

clean:
	@git clean -X -d -f
