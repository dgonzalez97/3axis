CC ?= cc
CFLAGS := -std=c11 -O2
WARNINGS := -Wall -Wextra -Wpedantic -Wconversion -Wshadow
CPPFLAGS := -Iinclude
LDLIBS := -lm

TARGET := build/rate_damping_testbench
SOURCES := src/rate_damping.c tests/testbench.c

.PHONY: all test clean

all: $(TARGET)

$(TARGET): $(SOURCES) include/rate_damping.h
	mkdir -p build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARNINGS) $(SOURCES) \
		-o $(TARGET) $(LDLIBS)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -rf build
