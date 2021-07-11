#include <fstream>
#include <cstdlib>
#include <algorithm>
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
  ra_radius = 20; // unit: m
  ra_upper_velocity = 10; // unit: m/s
  ra_lower_velocity = 0; // unit: m/s
  ra_safety_margin = 10; // unit: m
  ra_max_capacity = 0;
}

// read file //
bool
ra_mgr::read_vehicle(const string& infile)
{
  // read file //
  fstream fin(infile.c_str()); // To MobaXTerm, I need to use 'c_str()' to make it successfully compile
  
  string v_id;
  double eat, sa, da, vel; // angle is base on 360

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

    da = (sa > da)? da+360.0: da;
    
    // store //
    Vehicle* v = new Vehicle(v_id, eat, sa, da, vel);
    // intersection id
    vector<double>::iterator sp = find(ra_valid_source_angle.begin(), ra_valid_source_angle.end(), sa);
    int s_id = distance(ra_valid_source_angle.begin(), sp);
    vector<double>::iterator dp = find(ra_valid_destination_angle.begin(), ra_valid_destination_angle.end(), da);
    int d_id = distance(ra_valid_destination_angle.begin(), dp);
    v->source_intersection_id = s_id;
    v->destination_intersection_id = d_id;
    v_total.push_back(v);
    _vId2VehicleMap[v_id] = v;     
  }

  // sort v_total[i] accroding to earliest arrival time
  sort(v_total.begin(), v_total.end(), compare_v);
  cout << "=====================================" << endl;
  cout << "Earliest Arrival Time after Sorting : " << endl;
  cout << "=====================================" << endl;
  for ( int i = 0 ; i < v_total.size() ; i++)
    cout << v_total[i]->id << " : " << v_total[i]->earliest_arrival_time << " (s)" << endl;
  cout << endl;
  fin.close();
  return true;
}

bool
ra_mgr::read_ra_info(const string& rafile)
{
  fstream fin(rafile.c_str()); // To MobaXTerm, I need to use 'c_str()' to make it successfully compile
  fin >> ra_radius >> ra_lower_velocity >> ra_upper_velocity >> ra_safety_margin >> ra_max_capacity;

  // read ra_valid_source_angle //
  int num_of_entry, num_of_exit;
  fin >> num_of_entry;
  for(int i = 0; i < num_of_entry; i++){
    double tmp;
    fin >> tmp;
    ra_valid_source_angle.push_back(tmp);
    _sourceAngletoId[int(tmp)] = i; 
  }

  fin >> num_of_exit;
  for(int i = 0; i < num_of_exit; i++){
    double tmp;
    fin >> tmp;
    ra_valid_destination_angle.push_back(tmp);
    _destAngletoId[int(tmp)] = i;
  }

  fin.close();
  return true;
}

// utility //
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
  cerr << "Roundabout radius: " << ra_radius << " (m)" << endl;
  cerr << "Safety Upper Velocity: " << ra_upper_velocity << " (m/s)" << endl;
  cerr << "Safety Lower Velocity: " << ra_lower_velocity << " (m/s)" << endl;
  cerr << "Safety Margin: " << ra_safety_margin << " (m)" << endl;
  cerr << "Maximum Capacity: " << ra_max_capacity << " (unit)" << endl;
  cerr << "Valid Source Angle: ";
  for (int i=0; i < ra_valid_source_angle.size(); i++)
    cerr << ra_valid_source_angle[i] << " ";
  cerr << " (rad)" << endl;

  cerr << "Valid Destionation Angle: ";
  for (int i=0; i < ra_valid_destination_angle.size(); i++)
    cerr << ra_valid_destination_angle[i] << " ";
  cerr << " (rad)" << endl;
  cerr << "------------------------------------------------------------" << endl;
}

void
ra_mgr::Vehicle_information()
{
  cerr << "------------------------------------------------------------" << endl;
  cerr << "Vehicle information" << endl;
  for (int i=0; i < v_total.size(); i++)
  {
    cerr << "  Vehicle Id: " << v_total[i]->id << endl;
    cerr << "  Earliest Arrival Time: " << v_total[i]->earliest_arrival_time << endl;
    cerr << "  Source Angle: " << v_total[i]->source_angle << " (rad)" << endl;
    cerr << "  Denstination Angle: " << v_total[i]->destination_angle << " (rad)" << endl;
    cerr << "  Velocity:" << v_total[i]->velocity << " (m/s)" << endl;
    cerr << endl;
  }
  cerr << "------------------------------------------------------------" << endl;
   
}

void 
ra_mgr::output_solution(const string &path)
{
  ofstream fout(path.c_str());
  fout << v_total.size() << endl;
  if (_case == 5 || _case == 4)
  {
    DLnode* node;
    for(int i = 0; i < v_total.size(); i++){
        fout << v_total[i]->id << " ";
        node = v_total[i]->answer_head;
        fout << node->getStartTime() << " " << node->getStartAngle() << " ";
        while(1)
        {
          fout << node->getEndTime() << " " << node->getEndAngle() << " ";
          if (node == node->getFront()) break; // only one node in this answerList
          if (node->getFront() == v_total[i]->answer_head) break;
          // double check // 
          if (node->getEndTime() != node->getFront()->getStartTime()) // always occurs when an answerList end ( 0 -> 1 -> 2 -> 0)
          {
            cerr << "something wrong at answer!" << endl;
            cerr << node->getEndTime() << " / " << node->getFront()->getStartTime() << endl;
            return;
          }
          node = node->getFront();
        }
        fout << endl;
    }
  }
  else
  {
    // <t_1, angle_1>
    for(int i = 0; i < v_total.size(); i++){
        fout << v_total[i]->id << " ";
        for(int j = 0; j < v_total[i]->position.size(); j++){
            fout << v_total[i]->position[j].first << " " << ((v_total[i]->position[j].second>=360.0)? (v_total[i]->position[j].second-360): (v_total[i]->position[j].second) ) << " ";
        }
        fout << endl;
    }
  }
}

