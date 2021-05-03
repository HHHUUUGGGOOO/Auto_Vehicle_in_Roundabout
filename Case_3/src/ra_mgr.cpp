/*****************************************************************************************
  FileName     [ ra_mgr_1.cpp ]
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
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <algorithm>
#include "vehicle.h"
#include "ra_mgr.h"

using namespace std;

// sort vehicles by its earliest arrival time //
bool compare_v(Vehicle const *a, Vehicle const *b)
{
  return a->earliest_arrival_time < b->earliest_arrival_time;
}

// constructor //
ra_mgr::ra_mgr()
{ 
  ra_time_unit = 0.1; // unit: sec 
  ra_angle_unit =  degree_to_rad(1.5); // unit: 1.5 degree = 0.025 rad
  ra_radius = 20; // unit: m
  ra_safety_velocity = 7; // unit: m/s
  ra_safety_margin = 3; // unit: m
  ra_max_capacity = 0;
}


// read file //
bool
ra_mgr::read_vehicle(const string& infile)
{

  // read file //
  // char buffer[100];
  fstream fin(infile.c_str()); // To MobaXTerm, I need to use 'c_str()' to make it successfully compile
  
  int v_id = 0;
  float eat, sa, da, vel; // angle is base on 360

  while (fin >> v_id >> eat >> sa >> da >> vel){
    // check source/destination angle (5)(6) & base on 'π' // 

    // sa = degree_to_rad(sa);
    // da = degree_to_rad(da);
    if (sa >= 360 || da >= 360 || sa < 0 || da < 0) 
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not defined in 360 degree !!" << endl;
      return false;
    }
    else if (!verify_angle(sa, da))
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not ra_valid !!" << endl;
      return false;
    }
    
    da = (sa > da)? da+360: da;
    
    // store //
    Vehicle* v = new Vehicle(v_id, eat, sa, da, vel);
    v->safety_margin = round(v->velocity/2);
    //printf("%f, %f, Velocity = %f(rad)\n", v->velocity, ra_radius, v->velocity/ra_radius);
    v->angle_unit = v_min_angle_unit(v->velocity*ra_time_unit/ra_radius); //0.025*ceil((v->velocity/10)/0.5);
    // intersection id
    vector<float>::iterator sp = find(ra_valid_source_angle.begin(), ra_valid_source_angle.end(), sa);
    int s_id = distance(ra_valid_source_angle.begin(), sp);
    vector<float>::iterator dp = find(ra_valid_destination_angle.begin(), ra_valid_destination_angle.end(), da);
    int d_id = distance(ra_valid_destination_angle.begin(), dp);
    v->source_intersection_id = s_id;
    v->destination_intersection_id = d_id;
    v_total.push_back(v);     
  }

  // put vehicle into specific wait_list entry
  for (int i = 0 ; i < v_total.size() ; i++)
  {
    wait_list.push_back(v_total[i]);
  }

  // sort wait_list[i] accroding to earliest arrival time
  sort(wait_list.begin(), wait_list.end(), compare_v);
  cout << "=====================================" << endl;
  cout << "Earliest Arrival Time after Sorting : " << endl;
  cout << "=====================================" << endl;
  for ( int i = 0 ; i < v_total.size() ; i++)
  {
    cout << wait_list[i]->id << " -> " << wait_list[i]->earliest_arrival_time << " (s)" << endl;
  }
  cout << endl;
  fin.close();
  return true;
}

bool
ra_mgr::read_ra_info(const string& rafile)
{
  float r, sv, sm;
  int mc;
  string va;
  fstream fin(rafile.c_str()); // To MobaXTerm, I need to use 'c_str()' to make it successfully compile
  fin >> r >> sv >> sm >> mc;
  ra_radius = r;
  ra_safety_velocity = sv*1000/3600;
  ra_safety_margin = sm;
  ra_max_capacity = mc;

  // read ra_valid_source_angle //
  int num_of_entry, num_of_exit;
  fin >> num_of_entry;
  for(int i = 0; i < num_of_entry; i++){
    float tmp;
    fin >> tmp;
    ra_valid_source_angle.push_back(tmp);
  }
  // Resize the number of waiting queue
  waiting_lists.resize(num_of_entry);

  fin >> num_of_exit;
  for(int i = 0; i < num_of_exit; i++){
    float tmp;
    fin >> tmp;
    ra_valid_destination_angle.push_back(tmp);
  }

  fin.close();
  return true;
}

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
    vector<vector<pair<float, float>>> intersection_used_time(ra_valid_source_angle.size());
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
      for (int j = 0 ; j < (int)in_ra_time[i]/0.1 ; j++)
      {
        if (j == (int)in_ra_time[i]/0.1 - 0.1)
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


// utility //
bool
ra_mgr::check_conflict_by_wait(Vehicle const *tmp, vector<Vehicle*>& in_list)
{
  if (!in_list.size()) return false;
  return false;
}

bool                
ra_mgr::verify_angle(float sa, float da) 
{
  // sa: source angle, da: destination angle
  // printf("%f %f\n", sa, da);

  // Verify if sa is valid
  bool valid = false;
  for(int i=0; i < ra_valid_source_angle.size() && !valid; i++){
    if (sa == ra_valid_source_angle[i])
      valid=true; 
  }
  if(valid == false) return false;

  // Verify if da is valid
  valid = false;
  for(int i=0; i < ra_valid_destination_angle.size() && !valid; i++){
    if (da == ra_valid_destination_angle[i])
      valid=true;  
  }
  return valid;
}

void
ra_mgr::Roundabout_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Roundabout information" << endl;
  // cerr << "Purpose: " << ra_purpose << endl;
  cerr << "Roundabout ra_radius: " << ra_radius << " (m)" << endl;
  cerr << "Safety Velocity: " << ra_safety_velocity << " (m/s)" << endl;
  cerr << "Safety margin: " << ra_safety_margin << " (m)" << endl;
  cerr << "Maximum capacity: " << ra_max_capacity << " (unit)" << endl;
  cerr << "ra_valid source angles: ";
  for (int i=0; i < ra_valid_source_angle.size(); i++)
    cerr << ra_valid_source_angle[i] << " ";
  cerr << " (rad)" << endl;

  cerr << "ra_valid destionation angle: ";
  for (int i=0; i < ra_valid_destination_angle.size(); i++)
    cerr << ra_valid_destination_angle[i] << " ";
  cerr << " (rad)" << endl;
  cerr << "ra_time_unit: " << ra_time_unit << " (s)" << endl;
  cerr << "ra_angle_unit: " << ra_angle_unit << " (rad)"<<endl;
  cerr << "------------------------------------------------------------" << endl;
}

void
ra_mgr::Vehicle_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Vehicle information" << endl;
  for (int i=0; i < v_total.size(); i++)
  {
    cerr << "vehicle id: " << v_total[i]->id << endl;
    cerr << "  Earliest arrival time: " << v_total[i]->earliest_arrival_time << endl;
    cerr << "  Source_angle: " << v_total[i]->source_angle << " (rad)" << endl;
    cerr << "  Denstination_angle: " << v_total[i]->destination_angle << " (rad)" << endl;
    cerr << "  Velocity:" << v_total[i]->velocity << " (m/s)" << endl;
    cerr << "  Angle unit: " << v_total[i]->angle_unit << " (rad/time unit)" << endl;
    cerr << endl;
  }
  cerr << "------------------------------------------------------------" << endl;
   
}

void
ra_mgr::current_situation(vector<Vehicle*>& in_list, vector<Vehicle*>& wait_list) // need modify
{

  cerr << "------------------------------------------------------------" << endl;
  cerr << "Current situation" << endl;
  cerr << "In roundabout: " << endl;
  for (int i=0; i < in_list.size(); i++)
  {
    cerr << "    Vehicle id = " << in_list[i]->id << endl;
  }

  cerr << "Unscheduled vehicles: " << endl;
  for (int i=0; i < wait_list.size(); i++)
  {
    cerr << "    Vehicle id = " << wait_list[i]->id << endl;
  }
  cerr << "------------------------------------------------------------" << endl;
}

bool
ra_mgr::check_intersection(float angle)
{
  if (angle >= 360)
  {
    // select the equivalent angle in [0, 2*pi]
    angle = angle + 360*(int)(-angle/360);
  }
  for (int i=0; i < ra_valid_source_angle.size(); i++)
  {
    // if (angle >= 360) angle -= 360;
    if (angle == ra_valid_source_angle[i]) return true;
  }
  return false;
}

bool
ra_mgr::check_conflict(int index, vector<Vehicle*>& next_intersection, vector<Vehicle*>& in_list)
{
  //return true if conflict
  //remove conflict next_intersection
  if (next_intersection.size() > 0)
  {
    float before_angle = in_list[index]->now_angle;
    float after_angle = in_list[index]->now_angle + in_list[index]->angle_unit;
    float next_intersection_angle = 0;

    next_intersection_angle = (after_angle>360)? next_intersection[0]->source_angle+360:next_intersection[0]->source_angle;
    if (before_angle < next_intersection_angle && after_angle > next_intersection_angle)
    {
      if (next_intersection[0]->initial_priority > (in_list[index]->destination_angle - before_angle)) { return true; }
      else
      {
        next_intersection.erase(next_intersection.begin());
      }
    }
    return false;
  }
  return false;
}

void 
ra_mgr::output_solution(const string &path)
{
    ofstream fout(path.c_str());
    for(int i = 0; i < wait_list.size(); i++){
        fout << i << " ";
        for(int j = 0; j < wait_list[i]->position.size(); j++){
            fout << wait_list[i]->position[j].first << " " << wait_list[i]->position[j].second << " ";
        }
        fout << endl;
    }
    return;
}

