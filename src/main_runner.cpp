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
#include <sstream>
#include <cstring>
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
  char buffer[100];
  fstream fin(argv[1]);
  fin.getline(buffer, 25);
  // parsing parameters & store each vehicle's property
  int v_id, v_priority, v_depart_time, v_route_id;
  string v_color;
  while (fin >> v_id >> v_priority >> v_depart_time >> v_route_id >> v_color){
    Vehicle vehicle { v_id, v_priority, v_depart_time, v_route_id, v_color };
    All_Vehicle.push_back(vehicle);
  }

}