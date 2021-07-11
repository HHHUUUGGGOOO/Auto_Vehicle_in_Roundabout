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

#ifndef DELTA
#define DELTA 1e-6
#endif


inline double computeNeededTime(double _raRadius, double radiun, double velocity){
    return _raRadius * radiun / velocity;
}

void 
ra_mgr::line_trivial_solution_case_3()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }
    sa_size = ra_valid_source_angle.size();
    // cerr << "Debug message 0: sa_size:" <<sa_size<< endl;

    int v_size = v_total.size();
    vector<vector<pair<int, double> > > vehicle_answer_time_position_table(v_size);
    vector<vector<pair<double, double> > > intersection_used_time(ra_valid_source_angle.size());
    vector<double> intersection_latest_enter_time(ra_valid_source_angle.size(), 0.0);
    // Do while traversing all vehicles in the v_total
    for (int current_v_id = 0; current_v_id < v_size; current_v_id++)
    {
      // find first_start_time
      double sa = v_total[current_v_id]->source_angle;
      int enterAngleId = _sourceAngletoId[int(sa)];
      double da = (v_total[current_v_id]->destination_angle > 360)? (v_total[current_v_id]->destination_angle-360):v_total[current_v_id]->destination_angle;
      int exitAngleId = _destAngletoId[int(da)];
      double safety_time_margin = ra_safety_margin / v_total[current_v_id]->velocity;
      double vehicle_can_enter_time = max(v_total[current_v_id]->earliest_arrival_time, intersection_latest_enter_time[enterAngleId] + safety_time_margin);
      // printf("%d %lf\n", current_v_id, vehicle_can_enter_time);
      
      while(true){
      //   cerr << "Debug message 1" << endl;

        // Below _sourceAngletoId and _destAngletoId will conflict if they are not the same;
        

        {
          bool fail_flag = false;
          bool over_360_degree = false;
          // cerr << "Debug message 1.5: from intersection " << enterAngleId << " to intersection " << exitAngleId << endl;
          for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId++ ){
            // cerr << "Debug message 2: current angle id:" << currentAngleId << endl;

            if(currentAngleId == sa_size){
              currentAngleId = 0;
              over_360_degree = true;
            }
            if(currentAngleId == exitAngleId) { break; }
            
            double angleInterval = ra_valid_source_angle[currentAngleId] - v_total[current_v_id]->source_angle + ((over_360_degree)? 360:0);
            double move_time = computeNeededTime(ra_radius, degree_to_rad(angleInterval), v_total[current_v_id]->velocity);
            for(int j = 0; j < intersection_used_time[currentAngleId].size(); j++){
              if(vehicle_can_enter_time + move_time >= intersection_used_time[currentAngleId][j].first && 
                  vehicle_can_enter_time + move_time < intersection_used_time[currentAngleId][j].second){
                vehicle_can_enter_time = intersection_used_time[currentAngleId][j].second - move_time + DELTA;
                fail_flag = true;
                break;
              }
            }
            if(fail_flag) { break; }
          }
          // cerr << "Debug message 3" << endl;
          if(fail_flag) { continue; }
        }
        // cerr << "Debug message 4" << endl;
        {
          intersection_latest_enter_time[enterAngleId] = vehicle_can_enter_time;

          bool over_360_degree = false;
          double safety_margin_time = ra_safety_margin / v_total[current_v_id]->velocity;
          for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId ++){
            if(currentAngleId == sa_size){
              currentAngleId = 0;
              over_360_degree = true;
            }
            if(currentAngleId == exitAngleId) { break; }
            double angleInterval = ra_valid_source_angle[currentAngleId] - v_total[current_v_id]->source_angle + ((over_360_degree)? 360:0);
            double move_time = computeNeededTime(ra_radius, degree_to_rad(angleInterval), v_total[current_v_id]->velocity);
            vehicle_answer_time_position_table[current_v_id].push_back(make_pair(currentAngleId, vehicle_can_enter_time + move_time));
            intersection_used_time[currentAngleId].push_back(
                make_pair(vehicle_can_enter_time + move_time - safety_margin_time, 
                    vehicle_can_enter_time + move_time + safety_margin_time));
          }
          double angleInterval = ra_valid_source_angle[exitAngleId] - v_total[current_v_id]->source_angle + ((over_360_degree)? 360:0);
          double move_time = computeNeededTime(ra_radius, degree_to_rad(angleInterval), v_total[current_v_id]->velocity);
          vehicle_answer_time_position_table[current_v_id].push_back(make_pair(exitAngleId, vehicle_can_enter_time + move_time));
          //real_enter_time.push_back(v_total[0]->earliest_arrival_time);
          real_enter_time.push_back(vehicle_can_enter_time);
          break;
        }
      }
    }
    // debug 
    cerr << "====================" << endl;
    cerr << "Vehicle in ra time : " << endl;
    cerr << "====================" << endl;
    for ( int i = 0 ; i < v_size ; i++)
    {
      cerr << v_total[i]->id << " -> " << v_total[i]->earliest_arrival_time << " (s)" << endl;
    }
    cerr << endl;
    cerr << "=========================" << endl;
    cerr << "Vehicle real enter time : " << endl;
    cerr << "=========================" << endl;
    for ( int i = 0 ; i < real_enter_time.size() ; i++)
    {
      cerr << v_total[i]->id << " -> " << real_enter_time[i] << " (s)" << endl;
    }
  // make pair of position
    for (int i = 0 ; i < v_size ; i++)
    {
      // cerr << "Vehicle " << i << " :" << vehicle_answer_time_position_table[i].size() << endl;
      for (int j = 0 ; j < vehicle_answer_time_position_table[i].size(); j++)
      {
        // cerr << "angle: " <<  ra_valid_source_angle[vehicle_answer_time_position_table[i][j].first] << 
        //     " | time:" << vehicle_answer_time_position_table[i][j].second << endl;
        v_total[i]->position.push_back(
            make_pair(vehicle_answer_time_position_table[i][j].second, 
                ra_valid_source_angle[vehicle_answer_time_position_table[i][j].first] ));
        /*
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
        */
      }
    }
}
