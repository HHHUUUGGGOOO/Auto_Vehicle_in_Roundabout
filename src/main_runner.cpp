/*****************************************************************************************
  FileName     [ main_runner.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Main functions to simulate automatic vehicles' scheduling in roundabout ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdlib.h>
#include "main_runner.h"

using namespace std;

//----------------------------------------------------------------------------------------
//    Global Variable
//----------------------------------------------------------------------------------------
// Using 'vector' to store each 'Class Vehicle'
vector<Vehicle> All_Vehicle;

//----------------------------------------------------------------------------------------
//    Function
//----------------------------------------------------------------------------------------
// help message: handle 'openfile' error
void help_message()
{
  cout << "Please enter correct file path" << endl;
}

//----------------------------------------------------------------------------------------
//    Main Function
//----------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  // Openfile error
  if (argc != 3){
    help_message();
    return 0;
  }
  // Read the input file 
  fstream fin(argv[1]);
  

}