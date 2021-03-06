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
#include <ctime>
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
  // argv[1]:case index argv[2]:input file, argv[3]:input_ra_file, argv[4]:output_file
  if (argc == 5) 
  {
    if (!raMgr->read_ra_info(argv[3])) {
      cerr << "Error: cannot open roundabout information input file \"" << argv[3] << "\"!!\n";
      return 0;
    }
    if (!raMgr->read_vehicle(argv[2])) {
      cerr << "Error: cannot open vehicle information input file \"" << argv[2] << "\"!!\n";
      return 0;
    }
  }
  else 
  {
    cerr << "Error: illegal number of argument (" << argc << ")!!\n";
    return 0;
  }
  
  string in;
  while (1)
  {
    cerr << "Show roundabout information? (y/n)" << endl;
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

  while (1)
  {
    cerr << "Show vehicle information? (y/n)" << endl;
    cin >> in;
    if (in == "y")
    {
      raMgr->Vehicle_information();
      break;
    }
    else if (in == "n")
        break;
    else
      cerr << "unknown answer OAO!!" << endl;
  }


  ////// schedule //////
  while (1)
  {
    cerr << "Run schedule? (y/n)" << endl;
    cin >> in;
    if (in == "y")
    {
      // Calculate operation time
      clock_t start , end;
      start = clock();
      // Main algorithm
      if (strcmp(argv[1], "-case1") == 0)
      {
        cerr << "run case1 :>" << endl;
        raMgr->_case = 1;
        raMgr->line_trivial_solution_case_1();
      }
      else if (strcmp(argv[1], "-case2") == 0)
      {
        cerr << "run case2 :>" << endl;
        raMgr->_case = 2;
        raMgr->line_trivial_solution_case_2();
      }
      else if (strcmp(argv[1], "-case3") == 0)
      {
        cerr << "run case3 :>" << endl;
        raMgr->_case = 3;
        raMgr->line_trivial_solution_case_3();
      }
      else if (strcmp(argv[1], "-case4") == 0)
      {
        cerr << "run constant skyline solution case4 :>" << endl;
        raMgr->_case = 4;
        raMgr->constant_velocity_skyline_solution_case_4();
      }
      else if (strcmp(argv[1], "-case5") == 0)
      {
        cerr << "run acceleration solution :>" << endl;
        raMgr->_case = 5;
        raMgr->acceleration_solution_case_5();
      }
      // Calculate end time
      end = clock();
      double time = end - start;
      cout << "====" << endl;
      cout << "Time" << endl;
      cout << "====" << endl;
      cout << "time = " << time/CLOCKS_PER_SEC << " (sec)" <<endl;
      cout << endl;
      break;
    }
    else if (in == "n")
        break;
    else
      cerr << "unknown answer OAO!!" << endl;
  }

  ////// write output file //////
  raMgr->output_solution(argv[4]);

  delete raMgr;
  
  return 0;
}
