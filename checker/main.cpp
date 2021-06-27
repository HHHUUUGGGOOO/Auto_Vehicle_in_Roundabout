#include "checker.h"

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        cerr << "Error: illegal number of argument(" << argc << ")!!" << endl;
        return 0; 
    }

    Checker checker;
    checker.check(argv[1], argv[2], argv[3]);

    return 0;
}