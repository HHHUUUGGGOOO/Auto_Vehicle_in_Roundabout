/*****************************************************************************************
  FileName     [ ra_mgr.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Define basic variables for roundabout manager ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _RA_MGR_H
#define _RA_MGR_H

#include <vector>
#include <string>
#include <iostream>
#include <utility>
#include "vehicle.h"

using namespace std;

class ra_mgr;
extern ra_mgr* raMgr;



//----------------------------------------------------------------------------------------
//    Function
//----------------------------------------------------------------------------------------
class ra_mgr
{
  // typedef map<const int, Vehicle*>   VehicleMap;

  public:
    /* Default constructor */
    ra_mgr() {
      num_v_in_ra = 0;
    }
    virtual ~ra_mgr() {}

    /* Define function: read_file() */
    bool                read_vehicle(const string&);
    bool                read_ra_info(const string&);

    // utility //
    bool                verify_capacity();
    bool                verify_angle(float, float);
    void                Roundabout_information();
    void                current_situation();   
    bool                check_intersection(float); 
    bool                check_conflict(int, vector< pair<int, float> >&); 
    bool                check_capacity() {return (num_v_in_ra >= max_capacity);}    

    /* Define function: schedule() */
    void                do_scheduling();
    void                greedy_without_safetymargin();

    /* Define function: output_file() */
    void                do_output_file();

    /* Define variables */
    int                 num_v_in_ra; // number of vehicles in the roundabout now
    vector<Vehicle>     total_v; // store each vehicle's properties

    int                 v_unscheduled;
    float               time_unit; 
    float               angle_unit;
    vector <vector <int> >  output_chart; // store the output
    

    // roundabout information //
    float           radius;
    float           safety_velocity;
    float           safety_margin; // 小型車至少要保持「車速/2」距離(單位：公尺)；大型車至少要保持「車速-20」距離(單位：公尺)
    int             max_capacity;

    vector<float>   valid_source_angle; // 0 <= angle < 2*pi
    vector<float>   valid_destination_angle; // Si <= Di < Si + 2*pi ; D_i > 0
    
};

#endif