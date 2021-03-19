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
#include <cstring>
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
  if (argc == 4) 
  {
    if (!raMgr->read_ra_info(argv[2])) {
      cerr << "Error: cannot open roundabout information input file \"" << argv[2] << "\"!!\n";
      return 0;;
    }
    if (!raMgr->read_vehicle(argv[1])) {
      cerr << "Error: cannot open vehicle information input file \"" << argv[1] << "\"!!\n";
      return 0;
    }
  }
  else 
  {
    cerr << "Error: illegal number of argument (" << argc << ")!!\n";
    return 0;
  }

  string in;
  cerr << "Show roundabout information? (y/n)" << endl;
  while (1)
  {
    cin >> in;
    if (in == "y")
    {
      raMgr->Roundabout_information();
      break;
    }
    else if (in == "n")
        break;
    else
      cerr << "unknown answer OAO!!" << endl;
  }


  ////// schedule //////
  cerr << "Run schedule? (y/n)" << endl;
  while (1)
  {
    cin >> in;
    if (in == "y")
    {
      raMgr->greedy_without_safetymargin();
      break;
    }
    else if (in == "n")
        break;
    else
      cerr << "unknown answer OAO!!" << endl;
  }

  ////// write output file //////


  delete raMgr;

  return 0;
}