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
#include <utility>
#include "vehicle.h"
#include "ra_mgr.h"

using namespace std;

// constructor //
ra_mgr::ra_mgr()
{ 
  ra_time_unit = 0.1; // unit: sec
  ra_angle_unit = 0.025; // unit: rad
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

    // TODO: decide whether need to switch to 'π' form //
    /* 
    sa = sa / 360 * M_PI;
    da = da / 360 * M_PI;
    if (sa > da) da += M_PI; 
    */

    da = (sa > da)? da+360: da;
    
    // store //
    Vehicle* v = new Vehicle(v_id, eat, sa, da, vel);
    v->safety_margin = round(v->velocity/2);
    v->angle_unit = 0.025*ceil((v->velocity/10)/0.5);
    v_total.push_back(v);     
  }
  

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
  fin >> va;
  int current = 0; 
	int next;
	while (1)
	{
		next = va.find_first_of(",", current);
		if (next != current)
		{
			string tmp = va.substr(current, next - current);
			if (tmp.size() != 0)
				// ra_valid_source_angle.push_back(stoi(tmp));
        ra_valid_source_angle.push_back(atoi(tmp.c_str())); // To MobaXTerm, it cannot support C++11's 'stoi()'
		}
		if (next == string::npos) break;
		current = next + 1; 
	}

  // read ra_valid_destination_angle //
  fin >> va;
  current = 0; 
	while (1)
	{
		next = va.find_first_of(",", current);
		if (next != current)
		{
			string tmp = va.substr(current, next - current);
			if (tmp.size() != 0)
				// ra_valid_destination_angle.push_back(stoi(tmp));
        ra_valid_destination_angle.push_back(atoi(tmp.c_str()));
		}
		if (next == string::npos) break;
		current = next + 1; 
	}

  fin.close();
  
  // unit: m, r*θ >= 5 (小客車長度5公尺, 小型車至少要保持「車速/2」距離(單位：公尺)
  // 大型車至少要保持「車速-20」距離(單位：公尺)) 
  // ra_angle_unit = 360*(5/ra_radius) + 2*(ra_safety_velocity/2); 
  
  // find_ra_angle_unit(ra_radius, ra_safety_velocity)

 return true;
}


void                
ra_mgr::greedy_without_safetymargin()
{
  if (!v_total.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
    return;
  }

  float t = 0;
  int conflict_v;
  int angle;

  vector<int>  position_T(360/ra_angle_unit+1, 0); // for time t: position  
  vector<Vehicle*>  wait_list;
  vector<Vehicle*>  in_list;
  wait_list = v_total;
  vector< pair<int , Vehicle*> > trying_in;

  while(1)
  { 
    // cerr << "t = " << t << endl;
    // find trying in first //
    // cerr << "find trying in first..." << endl;
    for (int i=0; i < wait_list.size(); i++)
    {
      if (wait_list[i]->earliest_arrival_time <= t)
        trying_in.push_back(make_pair(i , wait_list[i])); // i means index in wait list
    }

    // cerr << "deal with vehicle in roundabout..." << endl;
    for (int i=0; i < in_list.size(); i++)
    {
      // leave //
      if (in_list[i]->now_angle+in_list[i]->angle_unit >= in_list[i]->destination_angle)
      {
        in_list[i]->position.push_back(make_pair(in_list[i]->destination_angle,t));
        in_list.erase(in_list.begin()+i);
        i--;
        continue;
      }
      // conflict and has lower priority // 
      // take check_intersection(v_total[i]->now_angle) out?
      else if(!check_conflict(i, trying_in, in_list))
      {
        in_list[i]->now_angle += in_list[i]->angle_unit;
        in_list[i]->position.push_back(make_pair(in_list[i]->now_angle,t));
        position_T[in_list[i]->now_angle/ra_angle_unit] = in_list[i]->id;
      }
    }

    // cerr << "send vehicle into roundabout..." << endl;
    int correction_term=0;
    for (int i=0; i < trying_in.size(); i++)
    {
      // check capacity
      if (in_list.size() >= ra_max_capacity) 
      {
        // cerr << "Capacity violation !!" << endl;
        break;
      }
      
      trying_in[i].second->position.push_back(make_pair(trying_in[i].second->source_angle,t));
      trying_in[i].second->now_angle = trying_in[i].second->source_angle; 
      position_T[trying_in[i].second->source_angle/ra_angle_unit] = trying_in[i].second->id;
      // check safety_margin with the previous vehicle
      int next_id = (i+1==trying_in.size()) ? 0: i+1;
      if ((trying_in[next_id].second->now_angle - (trying_in[i].second->now_angle + trying_in[i].second->angle_unit)) <= trying_in[i].second->safety_margin)
      {
        trying_in[next_id].second->angle_unit = trying_in[i].second->angle_unit;
      }
      
      
      in_list.push_back(trying_in[i].second);
      wait_list.erase(wait_list.begin()+trying_in[i].first-correction_term);
      correction_term++;
    }

    // cerr << "final..." << endl;
    output_chart.push_back(position_T);
    fill(position_T.begin(), position_T.end(), 0);
    t += ra_time_unit;
    vector < pair<int , Vehicle*> >().swap(trying_in);  // trying_in reset
    // current_situation(in_list, wait_list);
    if (!wait_list.size() && !in_list.size()) break;
  }

  // print chart
  for (int i=0; i < output_chart.size(); i++)
  {
    for (int j=0; j < output_chart[i].size(); j++)
    {
      if (output_chart[i][j] == 0) cerr << ". ";
      else cerr << output_chart[i][j] << " ";
    }
    cerr << endl;
  }
  cerr << endl;

}


// utility //

bool                
ra_mgr::verify_angle(float sa, float da) 
{
  // sa: source angle, da: destination angle
  for(int i=0; i < ra_valid_source_angle.size(); i++)
    if (sa == ra_valid_source_angle[i]) return true; 
  for(int i=0; i < ra_valid_destination_angle.size(); i++)
    if (da == ra_valid_destination_angle[i]) return true; 

  return false;
}

void
ra_mgr::Roundabout_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Roundabout information" << endl;
  cerr << "Roundabout ra_radius: " << ra_radius << " (m)" << endl;
  cerr << "Safety Velocity: " << ra_safety_velocity << " (m/s)" << endl;
  cerr << "Safety margin: " << ra_safety_margin << " (m)" << endl;
  cerr << "Maximum capacity: " << ra_max_capacity << " (unit)" << endl;
  cerr << "Angle unit" << ra_angle_unit << "(degree/time_unit)" << endl;
  cerr << "ra_valid source angles: ";
  for (int i=0; i < ra_valid_source_angle.size(); i++)
    cerr << ra_valid_source_angle[i] << " ";
  cerr << " (degree)" << endl;

  cerr << "ra_valid destionation angle: ";
  for (int i=0; i < ra_valid_destination_angle.size(); i++)
    cerr << ra_valid_destination_angle[i] << " ";
  cerr << " (degree)" << endl;
  cerr << "ra_time_unit: " << ra_time_unit << " (s)" << endl;
  cerr << "ra_angle_unit: " << ra_angle_unit << " (degree)"<<endl;
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
ra_mgr::check_intersection(float angle) // need modify
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
ra_mgr::check_conflict(int index, vector< pair<int, Vehicle*> >& trying_in, vector<Vehicle*>& in_list)
{
  //return true if conflict
  //remove conflict trying_in
  float before_angle = in_list[index]->now_angle;
  float after_angle = in_list[index]->now_angle + in_list[index]->angle_unit;
  float trying_in_angle = 0;
  for (int i=0; i < trying_in.size(); i++)
  {
    trying_in_angle = (after_angle>360)? trying_in[i].second->source_angle+360:trying_in[i].second->source_angle;
    if (before_angle < trying_in_angle && after_angle > trying_in_angle)
    {
      if (trying_in[i].second->initial_priority > in_list[index]->destination_angle - before_angle) return true;
      else
      {
        trying_in.erase(trying_in.begin()+i);
        i--;
      }
    }
  }
  return false;
}