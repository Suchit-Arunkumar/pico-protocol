CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror -O1 -g
SANFLAGS := -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer

SRC := firmware/test_pico_protocol.c firmware/pico_protocol.c
BIN := build/test_pico_protocol

.PHONY: test test-c test-python fuzz clean

test: test-c test-python

test-c: $(BIN)
	./$(BIN)

$(BIN): $(SRC) firmware/pico_protocol.h firmware/pico_protocol_rx.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SANFLAGS) -o $@ $(SRC) -lm

# libFuzzer harness for the receive path (clang only).
FUZZ_BIN := build/fuzz_rx

fuzz: $(FUZZ_BIN)

$(FUZZ_BIN): firmware/fuzz_rx.c firmware/pico_protocol.c firmware/pico_protocol.h firmware/pico_protocol_rx.h
	@mkdir -p build
	clang -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all \
	    -o $@ firmware/fuzz_rx.c firmware/pico_protocol.c -lm

test-python:
	python3 -m unittest discover -s python -v
	cd python && python3 example_roundtrip.py

clean:
	rm -rf build
