CXX=clang++
CFLAGS=-std=c++20 -Wall -pedantic
SRCS := $(wildcard *.cc)
MAMBA_ENV := $(shell python3 _getenv.py)
MAMBA_INCLUDE := $(MAMBA_ENV)/include
MAMBA_LIB := $(MAMBA_ENV)/lib/
LSEARCHPATH := -Wl,-rpath,$(MAMBA_LIB) -L$(MAMBA_LIB)
LFLAGS = -lSDL2 -lSDL2main -lopenblas
INCLUDE_DIR := -Iinclude -I$(MAMBA_INCLUDE)
OBJS := $(patsubst %.cc, %.o, $(SRCS))
EXE = 3dr.o

.PHONY: run clean

all: $(SRCS)
	$(CXX) $(CFLAGS) -o $(EXE) $(INCLUDE_DIR) $(LSEARCHPATH) $(SRCS) $(LFLAGS)

run:
	make clean; make; ./$(EXE)

clean:
	rm *.o *.out

