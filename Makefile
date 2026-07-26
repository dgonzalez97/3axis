CC ?= cc
CFLAGS := -std=c11 -O2
WARNINGS := -Wall -Wextra -Wpedantic -Wconversion -Wshadow
CPPFLAGS := -Iinclude
LDLIBS := -lm

RATE_TARGET := build/rate_damping_testbench
HEALTH_TARGET := build/health_monitor_testbench

RATE_SOURCES := src/rate_damping.c tests/testbench.c
HEALTH_SOURCES := src/health_monitor.c tests/health_testbench.c

.PHONY: all test clean

all: $(RATE_TARGET) $(HEALTH_TARGET)

$(RATE_TARGET): $(RATE_SOURCES) include/rate_damping.h
	mkdir -p build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARNINGS) $(RATE_SOURCES) \
		-o $(RATE_TARGET) $(LDLIBS)

$(HEALTH_TARGET): $(HEALTH_SOURCES) include/rate_damping.h \
		include/health_monitor.h
	mkdir -p build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(WARNINGS) $(HEALTH_SOURCES) \
		-o $(HEALTH_TARGET) $(LDLIBS)

test: all
	./$(RATE_TARGET)
	./$(HEALTH_TARGET)

clean:
	rm -rf build
