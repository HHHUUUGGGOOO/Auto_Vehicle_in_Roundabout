/*****************************************************************************************
  FileName     [ main.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Main function ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#include <iostream>
#include <cstdlib>
#include "ra_mgr.h"

using namespace std;

//----------------------------------------------------------------------------------------
//    Global Manager
//----------------------------------------------------------------------------------------
ra_mgr* raMgr = new ra_mgr();

//----------------------------------------------------------------------------------------
//    Main Function
//----------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
  ////// read input file //////
  if (argc == 3) {
    if (!raMgr->read_vehicle(argv[1])) {
      cerr << "Error: cannot open input file \"" << argv[1] << "\"!!\n";
    }
  }
  else {
    cerr << "Error: illegal number of argument (" << argc << ")!!\n";
  }

  ////// schedule //////
  raMgr->do_scheduling();

  ////// write output file //////



  return 0;
}