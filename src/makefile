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
bin/runner		:src/main_opt.o src/ra_mgr.o src/case1.o src/case2.o src/case3.o src/skylinesol.o
				$(CC) $(OPTFLAGS) src/main_opt.o src/ra_mgr.o src/case1.o src/case2.o src/case3.o src/skylinesol.o -o bin/runner
src/main_opt.o 		: src/main.cpp
				$(CC) $(CFLAGS) $< -Ilib -o $@
src/ra_mgr.o    	: src/ra_mgr.cpp src/ra_mgr.h 
				$(CC) $(CFLAGS) $< -Ilib -o $@
src/case1.o     	: src/case1.cpp src/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src/case2.o     	: src/case2.cpp src/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src/case3.o     	: src/case3.cpp src/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src/skylinesol.o    : src/skylinesol.cpp src/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@

# clean all the .o and executable files
clean:
		rm -rf src/*.o bin/*

