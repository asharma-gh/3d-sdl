CXX =clang++
CFLAGS =-std=c++20 -Wall -pedantic
SRCS :=$(wildcard *.cc)
MAMBA_ENV :=$(shell python3 _getenv.py)
MAMBA_INCLUDE :=$(MAMBA_ENV)/include
MAMBA_LIB :=$(MAMBA_ENV)/lib/
LSEARCHPATH :=-Wl,-rpath,$(MAMBA_LIB) -L$(MAMBA_LIB)
LFLAGS =-lSDL2 -lSDL2main -lopenblas
INCLUDE_DIR :=-Iinclude -I$(MAMBA_INCLUDE)
DEPS :=$(wildcard include/*.hh)
OBJS :=$(patsubst %.cc, %.o, $(SRCS))
OBJ_DIR :=bin
OBJ_LOC :=$(addprefix $(OBJ_DIR)/, $(OBJS))
EXE =3dr.o

.PHONY: run clean $(OBJS)

all: $(OBJS)
	$(CXX) $(CFLAGS) -o $(EXE) $(INCLUDE_DIR) $(LSEARCHPATH) $(OBJ_LOC) $(LFLAGS)

$(OBJ_DIR)/$(OBJS): $(SRCS)
	mkdir -p $(OBJ_DIR)
	$(CXX) $(CFLAGS) -o $(OBJ_DIR)/$@ $(INCLUDE_DIR) -c $<


run:
	make clean; make; ./$(EXE)

clean:
	rm *.o *.out

