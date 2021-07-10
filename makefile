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
bin/runner		:src_line_segment/main_opt.o src_line_segment/ra_mgr.o src_line_segment/case1.o src_line_segment/case2.o src_line_segment/case3.o src_line_segment/case4.o src_line_segment/case5_acceleration.o
				$(CC) $(OPTFLAGS) src_line_segment/main_opt.o src_line_segment/ra_mgr.o src_line_segment/case1.o src_line_segment/case2.o src_line_segment/case3.o src_line_segment/case4.o src_line_segment/case5_acceleration.o -o bin/runner
src_line_segment/main_opt.o 		: src_line_segment/main.cpp
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/ra_mgr.o    	: src_line_segment/ra_mgr.cpp src_line_segment/ra_mgr.h 
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/case1.o     	: src_line_segment/case1.cpp src_line_segment/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/case2.o     	: src_line_segment/case2.cpp src_line_segment/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/case3.o     	: src_line_segment/case3.cpp src_line_segment/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/case4.o    : src_line_segment/case4.cpp src_line_segment/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@
src_line_segment/case5_acceleration.o    : src_line_segment/case5_acceleration.cpp src_line_segment/ra_mgr.h
				$(CC) $(CFLAGS) $< -Ilib -o $@

# clean all the .o and executable files
clean:
		rm -rf src_line_segment/*.o bin/*

