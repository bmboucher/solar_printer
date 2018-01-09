CXX := g++
CFLAGS := -std=c++11 -Iinc -pthread -lwiringPi

HDRS := $(shell find inc -name '*.hpp')
SRCS := $(shell find src -name '*.cpp')
OBJS := $(SRCS:src/%.cpp=obj/%.o)
EXE  := bin/solar_printer

$(EXE): $(OBJS)
	@mkdir -p bin
	$(CXX) -o $(@) $(OBJS) $(CFLAGS)

obj/%.o: src/%.cpp $(HDRS)
	@mkdir -p obj
	$(CXX) -o $(@) -c $(<) $(CFLAGS)

all: solar_printer
clean:
	rm -f bin/* obj/*
