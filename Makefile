CC      ?= gcc
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -pedantic
CFLAGS  += -Iinclude
LDLIBS  = -lm

SRC = $(wildcard src/*.c)
OBJ = $(SRC:src/%.c=build/%.o)
BIN = traffic

.PHONY: all run export clean

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

build/%.o: src/%.c $(wildcard include/*.h) | build
	$(CC) $(CFLAGS) -c -o $@ $<

build:
	mkdir -p build

run: $(BIN)
	./$(BIN)

export: $(BIN)
	mkdir -p web/data
	./$(BIN) --export

clean:
	rm -rf build $(BIN)
