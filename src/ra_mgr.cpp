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

    // TODO: decide whether need to switch to PI form //
    /* 
    sa = sa / 360 * M_PI;
    da = da / 360 * M_PI;
    if (sa > da) da += M_PI; 
    */

    da = (sa > da)? da+360: da;
    // store //
    Vehicle vehicle(v_id, eat, sa, da);
    total_v.push_back(vehicle);  
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
  fstream fin(rafile);
  fin >> r >> sv >> sm >> mc;
  radius = r;
  safety_velocity = sv;
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

 return true;
}

void                
ra_mgr::greedy_without_safetymargin()
{
  /* unit 
  t = 100 ms from 0 to 5 s
  theta = 10 degree
  */
  if (!total_v.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
    return;
  }

  int time_unit=100;
  float angle_unit=30;

  int t=0;
  int conflict_v;
  int angle;

  vector< pair<int, float> > trying_in; // <index, source angle>
  while(1)
  {
    // find trying in first //
    for (int i=0; i<total_v.size(); i++)
    {
      if (total_v[i].status== WAIT && total_v[i].earliest_arrival_time <= t)
        trying_in.push_back(make_pair(i, total_v[i].source_angle));
    }

    for (int i=0; i<total_v.size(); i++)
    {
      if (total_v[i].status == IN)
      {
        total_v[i].now_angle+=angle_unit;
        // leave //
        if (total_v[i].now_angle==total_v[i].destination_angle)
          total_v[i].status = OUT;
        // conflict and has lower priority // 
        else if (check_intersection(total_v[i].now_angle) && check_conflict(i, trying_in))
          total_v[i].now_angle-=angle_unit;
        
        // no conflict or has higher priority -> do nothing //
        total_v[i].position.push_back(make_pair(total_v[i].now_angle,t));
      }
    }

    for (int i=0; i<trying_in.size(); i++)
    { 
      if (trying_in[i].first != -1)
      {
        total_v[trying_in[i].first].position.push_back(make_pair(total_v[trying_in[i].first].source_angle,t));
        total_v[trying_in[i].first].status=IN;
        total_v[trying_in[i].first].now_angle=total_v[trying_in[i].first].source_angle;
      }
    }
    t+=100;
    trying_in.clear();
    current_situation();
    if (t == 1100) break;
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

void
ra_mgr::Roundabout_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Roundabout information" << endl;
  cerr << "Roundabout radius: " << radius << " (m)" << endl;
  cerr << "Safety Velocity: " << safety_velocity << " (km/hr)" << endl;
  cerr << "Safety margin: " << safety_margin << " (m)" << endl;
  cerr << "Maximum capacity: " << max_capacity << " (unit)" << endl;
  cerr << "Valid source angles: ";
  for (int i=0; i<valid_source_angle.size(); i++)
    cerr << valid_source_angle[i] << " ";
  cerr << " (degree)" << endl;

  cerr << "Valid destionation angle: ";
  for (int i=0; i<valid_destination_angle.size(); i++)
    cerr << valid_destination_angle[i] << " ";
  cerr << " (degree)" << endl;
  cerr << "------------------------------------------------------------" << endl;
}

void
ra_mgr::current_situation()
{
  vector<int> wait;
  vector<int> out;

  cerr << "------------------------------------------------------------" << endl;
  cerr << "Current situation" << endl;
  for (int i=0; i< total_v.size(); i++)
  {
    if (total_v[i].status==IN)
      cerr << "Vehicle id: " << total_v[i].id << " -> " 
        << total_v[i].position.back().first << " degree (" << total_v[i].position.back().second << " ms)." << endl;
    else if (total_v[i].status==WAIT)
      wait.push_back(total_v[i].id);
    else if (total_v[i].status==OUT)
      out.push_back(total_v[i].id);    
  }

  cerr << "Unscheduled vehicles' id: ";
  for (int i=0; i< wait.size(); i++)
    cerr << wait[i] << " ";
  cerr << endl;

  cerr << "Scheduled vehicles' id: ";
  for (int i=0; i< out.size(); i++)
    cerr << out[i] << " ";
  cerr << endl;
  cerr << "------------------------------------------------------------" << endl;
}

bool
ra_mgr::check_intersection(float angle)
{
  for (int i=0; i < valid_source_angle.size(); i++)
  {
    if (angle >=360) angle-=360;
    if (angle==valid_source_angle[i]) return true;
  }
  return false;
}

bool
ra_mgr::check_conflict(int index, vector< pair<int, float> >& trying_in)
{
  for (int i=0; i< trying_in.size(); i++)
  {
    if (trying_in[i].second == total_v[i].now_angle || trying_in[i].second == total_v[i].now_angle-360)
    {
      if (total_v[trying_in[i].first].destination_angle- total_v[trying_in[i].first].source_angle> total_v[i].destination_angle-total_v[i].now_angle) return true;
      else
      {
        trying_in[i].first = -1;
        return false;
      }
    }
  }
  return false;
}