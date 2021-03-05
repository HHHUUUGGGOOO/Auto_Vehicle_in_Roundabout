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
#include "ra_mgr.h"

using namespace std;

// read file //
bool
ra_mgr::read_vehicle(const string& infile)
{
  // reset //
  num_v_in_ra = 0;
  total_v_info.clear();

  // read file //
  char buffer[100];
  fstream fin(infile);
  
  int v_id=0;
  float eat, sa, da; // angle is base on 360

  while (fin >> v_id >> eat >> sa >> da){
    // check source/destination angle (5)(6) & base on PI // 
    if (sa >= 360 || da >= 360) 
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not defined in 360 degree !!" << endl;
      return false;
    }
    else if (!verify_angle(sa, da))
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not valid !!" << endl;
      return false;
    }

    // need to consider whether switch to PI form //
    /* 
    sa = sa / 360 * M_PI;
    da = da / 360 * M_PI;
    if (sa > da) da += M_PI; 
    */

    // store //
    Vehicle vehicle(v_id, eat, sa, da);
    total_v_info.push_back(vehicle);  
  }

  fin.close();
  return true;
}

bool
ra_mgr::read_ra_info(const string& rafile)
{
  // fstream fin(rafile);
  // fin.close();

  radius = 10.0;
  safety_velocity=1.0;
  safety_margin=1.0;
  max_capacity=10;

  for (int i = 0 ; i < 4; i++)
  {
    valid_source_angle.push_back(i*90);
    valid_destination_angle.push_back(i*90);
    valid_destination_angle.push_back((i+4)*90);
 }

 return true;
}

void
ra_mgr::do_scheduling()
{

}

void                
ra_mgr::greedy_without_safetymargin()
{
  /* unit 
  t = 100 ms from 0 to 5 s
  theta = 10 degree
  */

  if (!total_v_info.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
    return;
  }

}

// utility //
bool                
ra_mgr::verify_angle(float sa, float da) 
{
  for(int i=0; i<valid_source_angle.size(); i++)
    if (sa == valid_source_angle[i]) return true; 
  for(int i=0; i<valid_destination_angle.size(); i++)
    if (da == valid_destination_angle[i]) return true; 

  return false;
}