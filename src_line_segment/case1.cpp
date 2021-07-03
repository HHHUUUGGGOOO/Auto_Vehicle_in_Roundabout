/*****************************************************************************************
  FileName     [ case1.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Case 1: First come First serve, next car can enter after the previous one is out ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
**

***************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------

#include <cstring>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <algorithm>
#include "vehicle.h"
#include "ra_mgr.h"

using namespace std;

void 
ra_mgr::line_trivial_solution_case_1()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }
    int n_vehicle = v_total.size();
  // in ra time
    for (int i = 0 ; i < n_vehicle ; i++)
    {
      double run_time = ra_radius*degree_to_rad(v_total[i]->destination_angle-v_total[i]->source_angle)/v_total[i]->velocity;
      // 無條件進入到小數點後第一位
      run_time = ceil(run_time*10 + 0.5)/10;
      in_ra_time.push_back(run_time);
    }
    // The first vehicle
    real_enter_time.push_back(v_total[0]->earliest_arrival_time);
    // Do while traversing all vehicles in the v_total
    for (int i = 1 ; i < n_vehicle ; i++)
    {
      double can_enter_time = real_enter_time[i-1] + in_ra_time[i-1];
      real_enter_time.push_back(max(v_total[i]->earliest_arrival_time, can_enter_time));
    }
    // debug 
    cout << "====================" << endl;
    cout << "Vehicle in ra time : " << endl;
    cout << "====================" << endl;
    for (int i = 0 ; i < in_ra_time.size() ; i++)
    {
      cerr << v_total[i]->id << " -> " << in_ra_time[i] << " (s)" << endl;
    }
    cout << endl;
    cout << "=========================" << endl;
    cout << "Vehicle real enter time : " << endl;
    cout << "=========================" << endl;
    for (int i = 0 ; i < real_enter_time.size() ; i++)
    {
      cerr << v_total[i]->id << " -> " << real_enter_time[i] << " (s)" << endl;
    }
  // make pair of position
    for (int i = 0 ; i < n_vehicle ; i++)
    {
      for (int j = 0 ; j < (int)(in_ra_time[i]/0.1) ; j++)
      {
        if (j == (int)(in_ra_time[i]/0.1) - 1)
        {
          double t = real_enter_time[i] + (ra_radius*degree_to_rad(v_total[i]->destination_angle-v_total[i]->source_angle)/v_total[i]->velocity);
          double angle = v_total[i]->destination_angle;
          v_total[i]->position.push_back(make_pair(t, angle));
        }
        else
        {
          double t = real_enter_time[i] + 0.1*j;
          double angle = v_total[i]->source_angle + rad_to_degree(v_total[i]->velocity*0.1*j/ra_radius);
          v_total[i]->position.push_back(make_pair(t, angle));
        }
      }
    }
}
