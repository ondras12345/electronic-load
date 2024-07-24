HEX_FILE = .pio/build/ATmega8/firmware.hex
## 		VARIABLES
## Set environment variables or specify on the command line to override.
## Either of these should work:
## $ SSH_HOST=test make upload_remote
## $ make upload_remote SSH_HOST=test
##
SSH_HOST ?= rpi                  ## SSH host for upload_remote
UPLOAD_PORT ?= /dev/ttyUSB-load  ## Serial port for upload via a REMOTE host


##
## 		TARGETS
.PHONY: all
all:            ## Compile, default target for Vim
all: compile_commands.json compile

.PHONY: compile
compile:        ## Compile firmware.
	pio run

.PHONY: clean
clean:          ## Clean project.
	pio run --target clean

.PHONY: upload
upload:         ## Upload firmware via bootloader.
	pio run --target upload

# if jump to bootloader command fails, try powering the device down and
# flipping the switch once avrdude starts
.PHONY: upload_remote
upload_remote:  ## Upload firmware via remote SSH_HOST.
	tar cf - $(HEX_FILE) -C ~/.platformio/packages/tool-avrdude/ avrdude.conf | \
	ssh $(strip $(SSH_HOST)) '\
	    set -e; \
	    tmpdir=$$(mktemp -d -t tmp.XXXXXXXXXX); \
	    cleanup () { \
	        rm -rf "$$tmpdir"; \
	    }; \
	    trap cleanup EXIT; \
	    cd "$$tmpdir"; \
	    tar xf -; \
	    stty -F $(UPLOAD_PORT) raw 9600 cs8 -parenb -cstopb -echo ; \
	    echo "\n*BOOTLOADER\n" > $(UPLOAD_PORT); \
	    read -t 0.1 garbage < $(UPLOAD_PORT) || true; \
	    set -o xtrace; \
	    avrdude -p atmega8 -C avrdude.conf -c arduino -b 115200 -D -P $(UPLOAD_PORT) -U flash:w:$(HEX_FILE):i \
	    '

.PHONY: check
check:          ## Run static code analysis
	pio check

.PHONY: test
test:           ## Run native tests.
	pio test -e native -v

compile_commands.json: platformio.ini
	pio run --target compiledb

.PHONY: help
help:           ## Show this help.
	@grep -F -h "##" $(MAKEFILE_LIST) | sed -e '/unique_BhwaDzu7C/d;s/\\$$//;s/##//'
