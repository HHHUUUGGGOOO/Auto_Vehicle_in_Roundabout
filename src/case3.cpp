/*****************************************************************************************
  FileName     [ case3.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Case 3:  ]
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

void 
ra_mgr::line_trivial_solution_case_3()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }
    int n_vehicle = wait_list.size();
  // in ra time

    for (int i = 0 ; i < n_vehicle ; i++)
    {
      float run_time = ra_radius*degree_to_rad(wait_list[i]->destination_angle-wait_list[i]->source_angle)/wait_list[i]->velocity;
      // 無條件進入到小數點後第一位
      run_time = ceil(run_time*10 + 0.5)/10;
      in_ra_time.push_back(run_time);
    }
	
	// intersection
    vector<vector<pair<float, float> > > intersection_used_time(ra_valid_source_angle.size());
    // Do while traversing all vehicles in the wait_list
    for (int i = 0 ; i < n_vehicle ; i++)
    {
      // find first_start_time
      float vehicle_can_enter_time = wait_list[i]->earliest_arrival_time;
      printf("%d %f\n", i, vehicle_can_enter_time);
      while(true){
        int over_360_degree = 0;
        float cur_angle = wait_list[i]->source_angle;
        int cur_intersection_id = wait_list[i]->source_intersection_id;
        bool finish_flag = true;
        while(cur_angle <= wait_list[i]->destination_angle){            
          float move_time = ra_radius * degree_to_rad(cur_angle - wait_list[i]->source_angle) / wait_list[i]->velocity;
		  for(int j = 0; j < intersection_used_time[cur_intersection_id].size(); j++){
			if(vehicle_can_enter_time + move_time >= intersection_used_time[cur_intersection_id][j].first && vehicle_can_enter_time + move_time < intersection_used_time[cur_intersection_id][j].second){
			  vehicle_can_enter_time = intersection_used_time[cur_intersection_id][j].second - move_time;
			  finish_flag = false;
			}
		  }
		  if(!finish_flag) break;

          if(cur_intersection_id == ra_valid_source_angle.size() - 1 && over_360_degree) { break; }
          else if(cur_intersection_id == ra_valid_source_angle.size()-1){
            cur_intersection_id = 0;
            over_360_degree = 1;
          }else{
            cur_intersection_id += 1;
          }
          cur_angle = ra_valid_source_angle[cur_intersection_id] + 360*over_360_degree;
        }
        if(finish_flag){
          cur_intersection_id = wait_list[i]->source_intersection_id;
          cur_angle = ra_valid_source_angle[cur_intersection_id];
          over_360_degree = 0;
          float safety_margin_time = 0.5;
          while( cur_angle <= wait_list[i]->destination_angle){
            float move_time = ra_radius * degree_to_rad(cur_angle - wait_list[i]->source_angle) / wait_list[i]->velocity;
            intersection_used_time[cur_intersection_id].push_back(make_pair(vehicle_can_enter_time + move_time - safety_margin_time, vehicle_can_enter_time + move_time + safety_margin_time));
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
      //real_enter_time.push_back(wait_list[0]->earliest_arrival_time);
      real_enter_time.push_back(vehicle_can_enter_time);
    }
    // debug 
    cout << "====================" << endl;
    cout << "Vehicle in ra time : " << endl;
    cout << "====================" << endl;
    for ( int i = 0 ; i < in_ra_time.size() ; i++)
    {
      cerr << wait_list[i]->id << " -> " << in_ra_time[i] << " (s)" << endl;
    }
    cout << endl;
    cout << "=========================" << endl;
    cout << "Vehicle real enter time : " << endl;
    cout << "=========================" << endl;
    for ( int i = 0 ; i < real_enter_time.size() ; i++)
    {
      cerr << wait_list[i]->id << " -> " << real_enter_time[i] << " (s)" << endl;
    }
  // make pair of position
    for (int i = 0 ; i < n_vehicle ; i++)
    {
      for (int j = 0 ; j < (int)(in_ra_time[i]/0.1) ; j++)
      {
        if (j == (int)(in_ra_time[i]/0.1) - 1)
        {
          float t = real_enter_time[i] + (ra_radius*degree_to_rad(wait_list[i]->destination_angle-wait_list[i]->source_angle)/wait_list[i]->velocity);
          float angle = wait_list[i]->destination_angle;
          wait_list[i]->position.push_back(make_pair(t, angle));
        }
        else
        {
          float t = real_enter_time[i] + 0.1*j;
          float angle = wait_list[i]->source_angle + rad_to_degree(wait_list[i]->velocity*0.1*j/ra_radius);
          wait_list[i]->position.push_back(make_pair(t, angle));
        }
      }
    }
}