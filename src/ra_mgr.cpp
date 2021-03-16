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

// read file //
bool
ra_mgr::read_vehicle(const string& infile)
{
  // reset //
  num_v_in_ra = 0;
  total_v.clear();

  // read file //
  char buffer[100];
  fstream fin(infile);
  
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
      cerr << "ID = " << v_id << "'s source or destination angle is not valid !!" << endl;
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
    Vehicle vehicle(v_id, eat, sa, da, vel);
    total_v.push_back(vehicle);     
  }
  
  v_unscheduled = total_v.size();

  fin.close();
  return true;
}

bool
ra_mgr::read_ra_info(const string& rafile)
{
  float r, sv, sm;
  int mc;
  string va;
  fstream fin(rafile);
  fin >> r >> sv >> sm >> mc;
  radius = r;
  safety_velocity = sv*1000/3600;
  safety_margin = sm;
  max_capacity = mc;

  // read valid_source_angle //
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
				valid_source_angle.push_back(stoi(tmp));
		}
		if (next == string::npos) break;
		current = next + 1; 
	}

  // read valid_destination_angle //
  fin >> va;
  current = 0; 
	while (1)
	{
		next = va.find_first_of(",", current);
		if (next != current)
		{
			string tmp = va.substr(current, next - current);
			if (tmp.size() != 0)
				valid_destination_angle.push_back(stoi(tmp));
		}
		if (next == string::npos) break;
		current = next + 1; 
	}

  fin.close();
  // unit: m, r*θ >= 5 (小客車長度5公尺, 小型車至少要保持「車速/2」距離(單位：公尺)
  // 大型車至少要保持「車速-20」距離(單位：公尺)) 
  // angle_unit = 360*(5/radius) + 2*(safety_velocity/2); 
  angle_unit=10;

 return true;
}


void                
ra_mgr::greedy_without_safetymargin()
{
  if (!total_v.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
    return;
  }

  float t = 0;
  time_unit = 0.1; 
  int conflict_v;
  int angle;

  vector< pair<int, float> > trying_in; // <index, source angle>
  vector<int>  position_T(360/angle_unit+1, 0); // for time t: position  
  while(1)
  {
    // find trying in first //
    for (int i=0; i < total_v.size(); i++)
    {
      if (total_v[i].status == WAIT && total_v[i].earliest_arrival_time <= t)
        trying_in.push_back(make_pair(i, total_v[i].source_angle));
    }

    for (int i=0; i < total_v.size(); i++)
    {
      if (total_v[i].status == IN)
      {
        total_v[i].now_angle += angle_unit;
        // leave //
        if (total_v[i].now_angle == total_v[i].destination_angle)
        {
          total_v[i].status = OUT;
          num_v_in_ra--;
          v_unscheduled--;
        }
        // conflict and has lower priority // 
        // take check_intersection(total_v[i].now_angle) out?
        else if (!check_capacity() && check_conflict(i, trying_in))
          total_v[i].now_angle -= angle_unit;
        
        // no conflict or has higher priority -> do nothing //
        total_v[i].position.push_back(make_pair(total_v[i].now_angle,t));
        if (total_v[i].status != OUT)
          position_T[total_v[i].now_angle/angle_unit] = total_v[i].id;
      }
    }

    for (int i=0; i < trying_in.size(); i++)
    {
      if (check_capacity()) 
      {
        // cerr << "Capacity violation !!" << endl;
        break;
      }
      if (trying_in[i].first != -1)
      {
        total_v[trying_in[i].first].position.push_back(make_pair(total_v[trying_in[i].first].source_angle,t));
        total_v[trying_in[i].first].status = IN;
        total_v[trying_in[i].first].now_angle = total_v[trying_in[i].first].source_angle; 
        position_T[total_v[trying_in[i].first].source_angle/angle_unit] = total_v[trying_in[i].first].id;
        num_v_in_ra ++;
      }
    }
    output_chart.push_back(position_T);
    fill(position_T.begin(), position_T.end(), 0);
    t += time_unit;
    trying_in.clear();
    // current_situation();
    // if (t > 0.2) break;
    if (!v_unscheduled) break;
  }

  // print chart
  for (auto i=0; i < output_chart.size(); i++)
  {
    for (auto j=0; j < output_chart[i].size(); j++)
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
ra_mgr::verify_capacity()
{
  // Determine if vehicle number in ra now "+" this vehicle will exceed the max capacity
  return (num_v_in_ra + 1 > max_capacity) ? true: false;
}

bool                
ra_mgr::verify_angle(float sa, float da) 
{
  // sa: source angle, da: destination angle
  for(int i=0; i < valid_source_angle.size(); i++)
    if (sa == valid_source_angle[i]) return true; 
  for(int i=0; i < valid_destination_angle.size(); i++)
    if (da == valid_destination_angle[i]) return true; 

  return false;
}

void
ra_mgr::Roundabout_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Roundabout information" << endl;
  cerr << "Roundabout radius: " << radius << " (m)" << endl;
  cerr << "Safety Velocity: " << safety_velocity << " (m/s)" << endl;
  cerr << "Safety margin: " << safety_margin << " (m)" << endl;
  cerr << "Maximum capacity: " << max_capacity << " (unit)" << endl;
  cerr << "Valid source angles: ";
  for (int i=0; i < valid_source_angle.size(); i++)
    cerr << valid_source_angle[i] << " ";
  cerr << " (degree)" << endl;

  cerr << "Valid destionation angle: ";
  for (int i=0; i < valid_destination_angle.size(); i++)
    cerr << valid_destination_angle[i] << " ";
  cerr << " (degree)" << endl;
  cerr << "time_unit: " << time_unit << " (s)" << endl;
  cerr << "angle_unit: " << angle_unit << " (degree)"<<endl;
  cerr << "------------------------------------------------------------" << endl;
}

void
ra_mgr::current_situation()
{
  vector<int> wait;
  vector<int> out;

  cerr << "------------------------------------------------------------" << endl;
  cerr << "Current situation" << endl;
  for (int i=0; i < total_v.size(); i++)
  {
    if (total_v[i].status == IN)
      cerr << "Vehicle id: " << total_v[i].id << " -> " 
        << total_v[i].position.back().first << " degree (" << total_v[i].position.back().second << " ms)." << endl;
    else if (total_v[i].status == WAIT)
      wait.push_back(total_v[i].id);
    else if (total_v[i].status == OUT)
      out.push_back(total_v[i].id);    
  }

  cerr << "Unscheduled vehicles' id: ";
  for (int i=0; i < wait.size(); i++)
    cerr << wait[i] << " ";
  cerr << endl;

  cerr << "Scheduled vehicles' id: ";
  for (int i=0; i < out.size(); i++)
    cerr << out[i] << " ";
  cerr << endl;
  
  cerr << "Number of vehicles in roundabout now:" << num_v_in_ra << "/" << max_capacity << endl;
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
  for (int i=0; i < valid_source_angle.size(); i++)
  {
    // if (angle >= 360) angle -= 360;
    if (angle == valid_source_angle[i]) return true;
  }
  return false;
}

bool
ra_mgr::check_conflict(int index, vector< pair<int, float> >& trying_in)
{
  for (int i=0; i < trying_in.size(); i++)
  {
    if (trying_in[i].second == total_v[i].now_angle || trying_in[i].second == total_v[i].now_angle-360)
    {
      if (total_v[trying_in[i].first].destination_angle - total_v[trying_in[i].first].source_angle > total_v[i].destination_angle - total_v[i].now_angle) return true;
      else
      {
        trying_in[i].first = -1;
        return false;
      }
    }
  }
  return false;
}