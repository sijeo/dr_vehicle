# Build the imulog binary→CSV converter on either host or Pi.
# Usage: make       (build)
#        make clean

CC       ?= gcc
CFLAGS   ?= -O2 -Wall -Wextra -Wpedantic -std=c11
LDFLAGS  ?=

BIN      := imulog_to_csv

all: $(BIN)

$(BIN): imulog_to_csv.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(BIN)

.PHONY: all clean
