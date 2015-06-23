build:
	particle compile beam_timer.cpp beam_timer.h  --saveTo firmware.bin

flash:
	particle flash tractortimer1 firmware.bin

flash-usb:
	sudo particle flash --usb firmware.bin

