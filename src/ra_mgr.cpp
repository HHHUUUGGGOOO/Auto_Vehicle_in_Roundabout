/*****************************************************************************************
  FileName     [ ra_mgr.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Read file, do scheduling, and output answer ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "vehicle.h"
// #include "ra_info.h"
#include "ra_mgr.h"

using namespace std;

bool
ra_mgr::read_vehicle(const string& infile)
{
  char buffer[100];
  fstream fin(infile);
  // Parsing parameters & store each vehicle's property
  int v_id=0;
  float eat=5;
  float sa =0.5*M_PI;
  float da =1.5*M_PI;

  while (fin >> v_id >> eat >> sa >> da){
    Vehicle vehicle(v_id, eat, sa, da);
    total_v_info.push_back(vehicle);  
  }
  fin.close();
  return true;
}

bool
ra_mgr::do_scheduling()
{
  return false;
}