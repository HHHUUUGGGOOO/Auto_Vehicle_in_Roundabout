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
bin/runner	: roundabout.o main_opt.o 
			$(CC) $(OPTFLAGS) main_opt.o -o bin/runner
main_opt.o 	   	: src/main_runner.cpp src/main_runner.h
			$(CC) $(CFLAGS) $< -Ilib -o $@
roundabout.o    : src/roundabout.cpp src/roundabout.h
			$(CC) $(CFLAGS) $< -Ilib -o $@

# clean all the .o and executable files
clean:
		rm -rf *.o lib/*.a lib/*.o bin/*

