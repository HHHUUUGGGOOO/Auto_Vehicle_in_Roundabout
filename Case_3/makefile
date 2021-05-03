# CC and CFLAGS are varilables
CC = g++
CFLAGS = -c
AR = ar
ARFLAGS = rcv
# -c option ask g++ to compile the source files, but do not link.
# -g option is for debugging version
# -O2 option is for optimized version
OPTFLAGS = -O2

all	: bin/runner
	@echo -n ""

# optimized version
bin/runner	: main_opt.o ra_mgr.o  
			$(CC) $(OPTFLAGS) main_opt.o ra_mgr.o -o bin/runner
main_opt.o 	: src/main.cpp
			$(CC) $(CFLAGS) $< -Ilib -o $@
ra_mgr.o    : src/ra_mgr.cpp src/ra_mgr.h 
			$(CC) $(CFLAGS) $< -Ilib -o $@

# clean all the .o and executable files
clean:
		rm -rf *.o lib/*.a lib/*.o bin/*

