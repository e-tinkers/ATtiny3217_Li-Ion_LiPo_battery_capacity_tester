MCU=attiny3217
F_CPU=5000000UL

CFLAG = ../drivers/inc/
VPATH = ../drivers/src/

# BUILD_FLAGS=-Wl,-u,vfprintf -lprintf_flt -lm

ifneq (, $(filter $(F_CPU), 20000000UL 10000000UL 5000000UL))
	CFUSE = fuse2:w:0x02:m
endif
ifneq (, $(filter $(F_CPU), 16000000UL 8000000UL 4000000UL 2000000UL 1000000UL))
	CFUSE = fuse2:w:0x01:m
endif

clean: main.elf
	rm main.elf

build: main.c
	avr-gcc -I${CFLAG} -mmcu=$(MCU) -DF_CPU=$(F_CPU)  ${BUILD_FLAGS} -Os main.c ${VPATH}*.c -o main.elf

flash: main.elf
	avrdude -c serialupdi -P /dev/tty.usbserial-* -p $(MCU) -U $(CFUSE) -U flash:w:main.elf

all: build flash

