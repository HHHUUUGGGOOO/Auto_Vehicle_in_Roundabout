# CC and CFLAGS are varilables
CC = g++
CFLAGS = -c
AR = ar
ARFLAGS = rcv
# -c option ask g++ to compile the source files, but do not link.
# -g option is for debugging version
# -O2 option is for optimized version
OPTFLAGS = -O2

all	: ../bin/runner
	@echo -n ""

# optimized version
../bin/runner	: main_opt.o ra_mgr.o case1.o case2.o case3.o skylinesol.o
			$(CC) $(OPTFLAGS) main_opt.o ra_mgr.o case1.o case2.o case3.o skylinesol.o -o ../bin/runner
main_opt.o 	: main.cpp
			$(CC) $(CFLAGS) $< -Ilib -o $@
ra_mgr.o    : ra_mgr.cpp ra_mgr.h 
			$(CC) $(CFLAGS) $< -Ilib -o $@
case1.o     : case1.cpp ra_mgr.h
			$(CC) $(CFLAGS) $< -Ilib -o $@
case2.o     : case2.cpp ra_mgr.h
			$(CC) $(CFLAGS) $< -Ilib -o $@
case3.o     : case3.cpp ra_mgr.h
			$(CC) $(CFLAGS) $< -Ilib -o $@
skylinesol.o   : skylinesol.cpp ra_mgr.h
			$(CC) $(CFLAGS) $< -Ilib -o $@

# clean all the .o and executable files
clean:
		rm -rf *.o lib/*.a lib/*.o ../bin/*

