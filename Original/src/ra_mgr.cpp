/*****************************************************************************************
  FileName     [ ra_mgr.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Read file, do scheduling, and output answer ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
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
    sa = degree_to_rad(sa);
    da = degree_to_rad(da);
    if (sa >= 2*PI || da >= 2*PI || sa < 0 || da < 0) 
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not defined in 360 degree !!" << endl;
      return false;
    }
    else if (!verify_angle(sa, da))
    {
      cerr << "ID = " << v_id << "'s source or destination angle is not ra_valid !!" << endl;
      return false;
    }

    da = (sa > da)? da+2*PI: da;
    
    // store //
    Vehicle* v = new Vehicle(v_id, eat, sa, da, vel);
    v->safety_margin = round(v->velocity/2);
    //printf("%f, %f, Velocity = %f(rad)\n", v->velocity, ra_radius, v->velocity/ra_radius);
    v->angle_unit = v_min_angle_unit(v->velocity*ra_time_unit/ra_radius); //0.025*ceil((v->velocity/10)/0.5);
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
  int num_of_entry, num_of_exit;
  fin >> num_of_entry;
  for(int i = 0; i < num_of_entry; i++){
    float tmp;
    fin >> tmp;
    ra_valid_source_angle.push_back(degree_to_rad(tmp));
  }
  // Resize the number of waiting queue
  waiting_lists.resize(num_of_entry);

  fin >> num_of_exit;
  for(int i = 0; i < num_of_exit; i++){
    float tmp;
    fin >> tmp;
    ra_valid_destination_angle.push_back(degree_to_rad(tmp));
  }
  /*
  printf("Valid Source Angle:");
  for(int i = 0; i < ra_valid_source_angle.size(); i++)
    printf("%f ", ra_valid_source_angle[i]);
  printf("\n");
  printf("Valid Destination Angle:");
  for(int i = 0; i < ra_valid_destination_angle.size(); i++)
    printf("%f ", ra_valid_destination_angle[i]);
  printf("\n");
  */

  fin.close();
  
  // unit: m, r*θ >= 5 (小客車長度5公尺, 小型車至少要保持「車速/2」距離(單位：公尺)
  // 大型車至少要保持「車速-20」距離(單位：公尺)) 
  // ra_angle_unit = 360*(5/ra_radius) + 2*(ra_safety_velocity/2); 
  
  // find_ra_angle_unit(ra_radius, ra_safety_velocity)

 return true;
}

void ra_mgr::line_trivial_solution(){
    if (!v_total.size()){
        cerr << "There is no vehicles to schedule !!" << endl;
        return;
    }
    int n_vehicle = v_total.size();
    vector<vector<pair<float, float>>> source_banned_time(ra_valid_source_angle.size());
    

    // Do while not finished
    for (int i = 0; i < n_vehicle; i++) {
        // find a time to enter the roundabout
        float start_time = v_total[i]->earliest_arrival_time, start_angle = v_total[i]->source_angle, dest_angle = v_total[i]->destination_angle;
        int start_intersection = 0;
        //// get start intersection
        while (start_intersection < ra_valid_source_angle.size() && ra_valid_source_angle[start_intersection] != start_angle) 
            start_intersection += 1;
        if (start_intersection >= ra_valid_source_angle.size()) {
            cerr << "Something goes wrong in the source angle of vehicle " << start_intersection << endl;
            return;
        }
        //// Check manually
        bool over_2PI = false;
        int intersection_idx = start_intersection;
        while(true){
            // Stop when the line segments between last ra entry and current ra entry covers the destination angle;
            float current_angle = ra_valid_source_angle[intersection_idx];
            bool ban_flag = false;
            while(current_angle <= dest_angle && !ban_flag){
                float move_time = (current_angle - start_angle) * ra_radius / v_total[i]->velocity;
                float arrival_time = start_time + move_time;
                for(vector<pair<float, float>>::iterator iter = source_banned_time[intersection_idx].begin(); iter != source_banned_time[intersection_idx].end(); iter++){
                    if(iter->first <= arrival_time && iter->second > arrival_time){
                        ban_flag = true;
                        start_time = iter->second - move_time;
                    } else if( iter->first > start_time) { break; }
                }
                if(ban_flag) { break; }
                // next intersection index
                if (intersection_idx == ra_valid_source_angle.size()-1) {
                    over_2PI = true;
                    intersection_idx = 0;
                } else {
                    intersection_idx += 1;
                }
                current_angle = (over_2PI)? ra_valid_source_angle[intersection_idx] + 2*PI : ra_valid_source_angle[intersection_idx];
            }
            if(!ban_flag){
                break;
            }
        }
        float end_time = start_time + (dest_angle - start_angle) * ra_radius;
        v_total[i]->position.push_back(make_pair(start_time, start_angle));
        v_total[i]->position.push_back(make_pair(end_time, dest_angle));
        // ban the intersections that will encounter this vehicle
        over_2PI = false;
        intersection_idx = start_intersection;
        float current_angle = ra_valid_source_angle[intersection_idx];
        float ban_range = ra_safety_margin / v_total[i]->velocity;
        while(current_angle <= dest_angle){
            float move_time = (current_angle - start_angle) * ra_radius / v_total[i]->velocity;
            float arrival_time = start_time + move_time;
            float upper_bound = arrival_time + ban_range, lower_bound = arrival_time - ban_range;
            bool last_flag = true;
            int insert_pos = 0;
            while(insert_pos < source_banned_time[intersection_idx].size() && last_flag){
                if( source_banned_time[intersection_idx][insert_pos].second >= lower_bound && source_banned_time[intersection_idx][insert_pos].first <= lower_bound){
                    source_banned_time[intersection_idx][insert_pos].second = max(upper_bound, source_banned_time[intersection_idx][insert_pos].second);
                    last_flag = false;
                } else if( source_banned_time[intersection_idx][insert_pos].first <= upper_bound && source_banned_time[intersection_idx][insert_pos].second >= upper_bound){
                    source_banned_time[intersection_idx][insert_pos].first = min(lower_bound, source_banned_time[intersection_idx][insert_pos].first);
                    last_flag = false;
                } else if( source_banned_time[intersection_idx][insert_pos].second < lower_bound){
                    source_banned_time[intersection_idx].insert(source_banned_time[intersection_idx].begin()+insert_pos, make_pair(lower_bound, upper_bound));
                }
                if ( last_flag ) { insert_pos += 1; }
            }
            if(last_flag){
                source_banned_time[intersection_idx].push_back(make_pair(lower_bound, upper_bound));
            } else{
                for(int j = 0; j < source_banned_time.size() - 1; j++){
                    if(source_banned_time[intersection_idx][j].second >= source_banned_time[intersection_idx][j+1].first){
                        if( source_banned_time[intersection_idx][j].second < source_banned_time[intersection_idx][j+1].second)
                            source_banned_time[intersection_idx][j].second = source_banned_time[intersection_idx][j+1].second;
                        source_banned_time[intersection_idx].erase(source_banned_time[intersection_idx].begin() + j + 1);
                    }
                }
            }

            if(intersection_idx == ra_valid_source_angle.size() -1){
                over_2PI = true;
                intersection_idx = 0;
            } else { intersection_idx += 1;}
            current_angle = ra_valid_source_angle[intersection_idx] + (over_2PI)? 2*PI:0;
        }
    }
}


void ra_mgr::trivial_solution(){
    if (!v_total.size()){
        cerr << "There is no vehicles to schedule !!" << endl;
        return;
    }
    float t = 0;


    vector<vector<int>> output_chart;
    int n_vehicle = v_total.size();
    int finished = 0;
    int start_entering = 0;
    
    //getchar();

    // Do while not finished
    while(finished < n_vehicle){
         cerr << "t = " << t << endl;
        // find trying in first //
        // cerr << "find trying in first..." << endl;
        //
        // Push vehicles to wait at its source road
        while(start_entering < n_vehicle && v_total[start_entering]->earliest_arrival_time <= t){
            for(int entering_road = 0; entering_road < ra_valid_source_angle.size(); entering_road++){
                if( v_total[start_entering]->source_angle == ra_valid_source_angle[entering_road]){
                    printf("Vehicles %d enters road %d (at %f)\n", start_entering, entering_road, ra_valid_source_angle[entering_road]);
                    waiting_lists[entering_road].push_back(v_total[start_entering]);
                    break;
                }
            }
            start_entering++;
        }

        // cerr << "deal with vehicle in roundabout..." << endl;

        for (vector<Vehicle*>::iterator iter = in_list.begin(); iter != in_list.end(); iter++){
          // leave //
            if ((*iter)->now_angle+(*iter)->angle_unit >= (*iter)->destination_angle){
                // Leaving time is calculated by 內插法
                float leaving_time = t + ra_time_unit * (((*iter)->destination_angle - (*iter)->now_angle)/(*iter)->angle_unit);
                (*iter)->position.push_back(make_pair(leaving_time, (*iter)->destination_angle));
                iter = in_list.erase(iter);
                finished++;
            }
            else{
                (*iter)->now_angle += (*iter)->angle_unit;
                (*iter)->position.push_back(make_pair(t, (*iter)->now_angle));
                printf("(%d, %f,%f) ", (*iter)->id, (*iter)->now_angle, (*iter)->destination_angle);
            }
            if(iter == in_list.end())
                break;
        }
        printf("\n");
        // cerr << "send vehicle into roundabout..." << endl;
        int correction_term=0;
        while(in_list.empty()){ //ra_max_capacity){
            int cur_choice = -1;
            float cur_waiting_time = -0.01; // Using waiting time as priority
            for(int i = 0; i < waiting_lists.size(); i++){
                if(waiting_lists[i].empty())
                    continue;
                float pr = t - waiting_lists[i][0]->earliest_arrival_time;
                if(pr > cur_waiting_time){
                    cur_choice = i;
                    cur_waiting_time = pr;
                }
            }
            // no vehicles
            if(cur_choice == -1) 
                break;
            // Check if conflict ( no need in trivial solution)
            // Push it into the roundabout
            Vehicle* Chosen = waiting_lists[cur_choice][0];
            Chosen->position.push_back( make_pair( t, Chosen->source_angle));
            Chosen->now_angle = Chosen->source_angle;
            // Find position to insert
            vector<Vehicle*>::iterator it = in_list.begin();
            while(it != in_list.end() && (*it)->now_angle < Chosen->source_angle){
                it++;
            }
            in_list.insert(it, Chosen);
            /*
            printf("in_list: ");
            for(int i = 0; i < in_list.size(); i++)
                printf("%f ", in_list[i]->now_angle);
            printf("\n");
            */
            // Pop
            waiting_lists[cur_choice].erase(waiting_lists[cur_choice].begin());
        }

        // cerr << "final..." << endl;
        cerr << "vehicles Enter: " << start_entering << ", vehicles in the roundabout: " << in_list.size() << ", vehicles finished: " << finished << endl;
        t += ra_time_unit;
        //printf("Press any key to continue\n");
        //getchar();
    }
}


void                
ra_mgr::greedy_without_safetymargin()
{
  if (!v_total.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
  }
  float t = 0;
  int conflict_v;
  int angle;

  vector<vector<int>> output_chart;
  vector<int>  position_T(360/ra_angle_unit+1, 0); // for time t: position  
  vector<Vehicle*>  wait_list;
  vector<Vehicle*>  in_list;
  wait_list = v_total;
 
  vector<Vehicle*> queue_0, queue_90, queue_180, queue_270;
  vector<vector <Vehicle*> > entry(ra_valid_source_angle.size());
  vector< pair<int , Vehicle*> > trying_in;

  while(1)
  { 
    // cerr << "t = " << t << endl;
    // find trying in first //
    // cerr << "find trying in first..." << endl;
    for (int i=0; i < wait_list.size(); i++)
    {
      if (wait_list[i]->earliest_arrival_time <= t)
        if (wait_list[i]->source_angle == (float) 0) { queue_0.push_back(wait_list[i]); }
        else if (wait_list[i]->source_angle == (float) 90) { queue_90.push_back(wait_list[i]); }
        else if (wait_list[i]->source_angle == (float) 180) { queue_180.push_back(wait_list[i]); }
        else if (wait_list[i]->source_angle == (float) 270) { queue_270.push_back(wait_list[i]); }
        // trying_in.push_back(make_pair(i , wait_list[i])); // i means index in wait list
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

      // check safety_margin with the previous vehicle
      int next_id = (i+1==in_list.size()) ? 0: i+1;
      if ((in_list[next_id]->now_angle - (in_list[i]->now_angle + in_list[i]->angle_unit)) <= in_list[i]->safety_margin)
      {
        in_list[next_id]->angle_unit = in_list[i]->angle_unit;
      }

      float next_intersection_angle = 0;
      vector<Vehicle*> next_intersection;
      if (((float) 0 <= in_list[i]->now_angle) && (in_list[i]->now_angle < (float) 90)) { next_intersection_angle = 90.0; next_intersection.assign(queue_90.begin(), queue_90.end()); }
      else if (((float) 90 <= in_list[i]->now_angle) && (in_list[i]->now_angle < (float) 180)) { next_intersection_angle = 180.0; next_intersection.assign(queue_180.begin(), queue_180.end()); }
      else if (((float) 180 <= in_list[i]->now_angle) && (in_list[i]->now_angle < (float) 270)) { next_intersection_angle = 270.0; next_intersection.assign(queue_270.begin(), queue_270.end()); }
      else if (((float) 270 <= in_list[i]->now_angle) && (in_list[i]->now_angle < (float) 360)) { next_intersection_angle = 0.0; next_intersection.assign(queue_0.begin(), queue_0.end()); }
      
      // conflict and in_list has lower priority // 

      // take check_intersection(v_total[i]->now_angle) out?

      // no conflict or conflict but in_list has higher priority
      if(!check_conflict(i, next_intersection, in_list))
      {
        in_list[i]->now_angle += in_list[i]->angle_unit;
        in_list[i]->position.push_back(make_pair(in_list[i]->now_angle,t));
        position_T[in_list[i]->now_angle/ra_angle_unit] = in_list[i]->id;
      }
    }

    // cerr << "send vehicle into roundabout..." << endl;
    int correction_term=0;
    for (int i=0; i < trying_in.size(); i++){
      // check capacity
      if (in_list.size() >= ra_max_capacity) 
      {
        // cerr << "Capacity violation !!" << endl;
        break;
      }
      
      trying_in[i].second->position.push_back(make_pair(trying_in[i].second->source_angle,t));
      trying_in[i].second->now_angle = trying_in[i].second->source_angle; 
      position_T[trying_in[i].second->source_angle/ra_angle_unit] = trying_in[i].second->id;      
      
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

/*
void                
ra_mgr::greedy_with_safetymargin()
{
  if (!v_total.size())
  {
    cerr << "There is no vehicles to schedule !!" << endl;
    return;
  }
  // float number has big problem
  vector<int>  position_at_time(floor(2*PI/ra_angle_unit)+1, 0); // the last one must be 2PI, the same as 0
  cerr << "size=" << position_at_time.size() << endl;
  float t=0;
  int i;
  vector<Vehicle*>  in_list;
  vector<vector <Vehicle*> > wait_list(ra_valid_source_angle.size());

  // put vehicle into specific wait_list entry
  for (i = 0; i < v_total.size(); i++)
    for(int j = 0; j<ra_valid_source_angle.size(); j++)
      if (v_total[i]->source_angle == ra_valid_source_angle[j])
        wait_list[j].push_back(v_total[i]);

  // sort wait_list[i] accroding to earlist arrival time
  for (i=0; i< wait_list.size(); i++)
  {
    sort(wait_list[i].rbegin(), wait_list[i].rend(), compare_v);
    *//* check entry
    cerr << i << endl;
    for (int j=0; j < wait_list[i].size(); j++)
    {
      cerr << wait_list[i][j]->id << " -> " << wait_list[i][j]->earliest_arrival_time << endl;
    }
    cerr << endl; *//*
  }

  Vehicle* tmp;
  int entry_index = -1;
  while(1)
  {
    cerr << "time = " << t << endl;

    // deal with vehicle in roundabout // 
    for (i = 0; i < in_list.size(); i++)
    {

      // leave //
      if (in_list[i]->now_angle+in_list[i]->angle_unit >= in_list[i]->destination_angle)
      {
        in_list[i]->position.push_back(make_pair(in_list[i]->destination_angle,t));
        in_list.erase(in_list.begin()+i);
        i--;
        continue;
      }

    }

    // deal with vehicle trying to enter //
    for (i = 0; i< wait_list.size(); i++)
    {
      if (wait_list[i].size())
      {
        tmp = wait_list[i].back();
        if (tmp->earliest_arrival_time <= t)
        {
          if (!check_conflict_by_wait(tmp, in_list)) 
          {
            cerr << tmp->id << " enter in " << tmp->source_angle << "(rad)" << endl;
            in_list.push_back(tmp);
            tmp->position.push_back(make_pair(tmp->source_angle, t));
            position_at_time[tmp->source_angle/ra_angle_unit]=tmp->id;
            wait_list[i].pop_back();
          }
        }
      }
    }

    t += ra_time_unit;
    if (t>0.05) break;
  }
  
}*/


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

void ra_mgr::output_solution(const string &path){
    ofstream fout(path.c_str());
    for(int i = 0; i < v_total.size(); i++){
        fout << i << " ";
        for(int j = 0; j < v_total[i]->position.size(); j++){
            fout << v_total[i]->position[j].first << " " << v_total[i]->position[j].second << " ";
        }
        fout << endl;
    }
    return;
}

