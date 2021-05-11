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
  double eat, sa, da, vel; // angle is base on 360

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
    //printf("%lf, %lf, Velocity = %lf(rad)\n", v->velocity, ra_radius, v->velocity/ra_radius);
    v->angle_unit = v_min_angle_unit(v->velocity*ra_time_unit/ra_radius); //0.025*ceil((v->velocity/10)/0.5);
    // intersection id
    vector<double>::iterator sp = find(ra_valid_source_angle.begin(), ra_valid_source_angle.end(), sa);
    int s_id = distance(ra_valid_source_angle.begin(), sp);
    vector<double>::iterator dp = find(ra_valid_destination_angle.begin(), ra_valid_destination_angle.end(), da);
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
  double r, sv, sm;
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
    double tmp;
    fin >> tmp;
    ra_valid_source_angle.push_back(tmp);
  }
  // Resize the number of waiting queue
  waiting_lists.resize(num_of_entry);

  fin >> num_of_exit;
  for(int i = 0; i < num_of_exit; i++){
    double tmp;
    fin >> tmp;
    ra_valid_destination_angle.push_back(tmp);
  }

  fin.close();
  return true;
}

// utility //
bool
ra_mgr::check_conflict_by_wait(Vehicle const *tmp, vector<Vehicle*>& in_list)
{
  if (!in_list.size()) return false;
  return false;
}

bool                
ra_mgr::verify_angle(double sa, double da) 
{
  // sa: source angle, da: destination angle
  // printf("%lf %lf\n", sa, da);

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
ra_mgr::check_intersection(double angle)
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
    double before_angle = in_list[index]->now_angle;
    double after_angle = in_list[index]->now_angle + in_list[index]->angle_unit;
    double next_intersection_angle = 0;

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
    // <t_1, angle_1>
    ofstream fout(path.c_str());
    for(int i = 0; i < wait_list.size(); i++){
        fout << wait_list[i]->id << " ";
        for(int j = 0; j < wait_list[i]->position.size(); j++){
            fout << wait_list[i]->position[j].first << " " << ((wait_list[i]->position[j].second>=360.0)? (wait_list[i]->position[j].second-360): (wait_list[i]->position[j].second) ) << " ";
        }
        fout << endl;
    }
    return;
}

