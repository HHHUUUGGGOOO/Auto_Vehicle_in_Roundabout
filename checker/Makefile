CFLAGS = -std=c++11 -g  

FILES = checker.o main.o

all : $(FILES)
	g++ $(CFLAGS) $(FILES) -lm -o checker
checker.o : checker.h checker.cpp
	g++ $(CFLAGS) -c checker.cpp
main.o : checker.h main.cpp
	g++ $(CFLAGS) -c main.cpp

clean:
	rm *.o checker	