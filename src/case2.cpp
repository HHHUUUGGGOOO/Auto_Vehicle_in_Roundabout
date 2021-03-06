/*****************************************************************************************
  FileName     [ case1.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Case 2:  ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
**

***************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------

#include <iostream>
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
ra_mgr::line_trivial_solution_case_2()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }
    int n_vehicle = v_total.size();
  // in ra time

    for (int i = 0 ; i < n_vehicle ; i++)
    {
      double run_time = ra_radius*degree_to_rad(v_total[i]->destination_angle-v_total[i]->source_angle)/v_total[i]->velocity;
      // 無條件進入到小數點後第一位
      // run_time = ceil(run_time*10 + 0.5)/10;
      in_ra_time.push_back(run_time);
    }

    // The first vehicle
    vector<double> intersection_can_enter_time(ra_valid_source_angle.size(), 0);
    // new added
    vector<double> intersection_max_time(ra_valid_source_angle.size(), 0);
    vector<bool> intersection_max_time_isExit(ra_valid_source_angle.size(), 1);
    // Do while traversing all vehicles in the v_total
    for (int i = 0 ; i < n_vehicle ; i++)
    {
      printf("%d \n", i);
      // find first_start_time
      //double can_enter_time = v_total[i]->earliest_arrival_time;
      double vehicle_can_enter_time = max(v_total[i]->earliest_arrival_time, intersection_can_enter_time[v_total[i]->source_intersection_id]);
      while(true){
        int over_360_degree = 0;
        double cur_angle = v_total[i]->source_angle;
        int cur_intersection_id = v_total[i]->source_intersection_id;

        double safety_margin_time = ra_safety_margin / v_total[i]->velocity;

        bool finish_flag = true;
        while(cur_angle < v_total[i]->destination_angle){            
          double move_time = ra_radius * degree_to_rad(cur_angle - v_total[i]->source_angle) / v_total[i]->velocity;
          // new added: consider max time at the intersection
          if (abs(vehicle_can_enter_time + move_time - intersection_max_time[cur_intersection_id]) < safety_margin_time) {
            if (intersection_max_time_isExit[cur_intersection_id] && (cur_intersection_id == v_total[i]->source_intersection_id)) {
              vehicle_can_enter_time = max(vehicle_can_enter_time, intersection_max_time[cur_intersection_id]);
            }
            else {
              vehicle_can_enter_time = max(vehicle_can_enter_time, intersection_max_time[cur_intersection_id] + safety_margin_time);
            }
          }

          if( vehicle_can_enter_time + move_time < intersection_can_enter_time[cur_intersection_id] ){
            vehicle_can_enter_time = max(vehicle_can_enter_time, (intersection_can_enter_time[cur_intersection_id] - move_time) + 1e-6);
            // finish_flag = false;
            // break;
          }

          if(cur_intersection_id == ra_valid_source_angle.size() - 1 && over_360_degree){ break; }
          else if(cur_intersection_id == ra_valid_source_angle.size() - 1){
            cur_intersection_id = 0;
            over_360_degree = 1;
          }else{
            cur_intersection_id += 1;
          }
          cur_angle = ra_valid_source_angle[cur_intersection_id] + 360*over_360_degree;
        }
        if(finish_flag){
          cur_intersection_id = v_total[i]->source_intersection_id;
          cur_angle = ra_valid_source_angle[cur_intersection_id];
          over_360_degree = 0;
          while( cur_angle < v_total[i]->destination_angle){
            double move_time = ra_radius * degree_to_rad(cur_angle - v_total[i]->source_angle) / v_total[i]->velocity;
            intersection_can_enter_time[cur_intersection_id] = vehicle_can_enter_time + move_time + safety_margin_time;
            // new added
            if ((vehicle_can_enter_time + move_time + safety_margin_time) > intersection_max_time[cur_intersection_id]) {
              intersection_max_time[cur_intersection_id] = vehicle_can_enter_time + move_time + safety_margin_time;
              // isExit
              if (cur_intersection_id != v_total[i]->destination_intersection_id) { intersection_max_time_isExit[cur_intersection_id] = false; }
              else { intersection_max_time_isExit[cur_intersection_id] = true; }
            }
            if(cur_intersection_id == ra_valid_source_angle.size() - 1 && over_360_degree){ break; }
            else if(cur_intersection_id == ra_valid_source_angle.size()-1){
              cur_intersection_id = 0;
              over_360_degree = 1;
            }else{
              cur_intersection_id += 1;
            }
            cur_angle = ra_valid_source_angle[cur_intersection_id] + 360*over_360_degree;
          }
          break;
        }

      }
      //real_enter_time.push_back(v_total[0]->earliest_arrival_time);
      real_enter_time.push_back(vehicle_can_enter_time);
    }
    // debug 
    cout << "====================" << endl;
    cout << "Vehicle in ra time : " << endl;
    cout << "====================" << endl;
    for ( int i = 0 ; i < in_ra_time.size() ; i++)
    {
      cerr << v_total[i]->id << " -> " << in_ra_time[i] << " (s)" << endl;
    }
    cout << endl;
    cout << "=========================" << endl;
    cout << "Vehicle real enter time : " << endl;
    cout << "=========================" << endl;
    for ( int i = 0 ; i < real_enter_time.size() ; i++)
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
