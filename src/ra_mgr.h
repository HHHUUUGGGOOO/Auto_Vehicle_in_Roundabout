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
    ra_mgr();
    virtual ~ra_mgr() {}

    // read info about roundabout and vehicle -> call in main //
    bool                read_vehicle(const string&);
    bool                read_ra_info(const string&);

    // utility -> call in schedule  //
    // bool                check_capacity() {return (v_in_ra_now >= ra_max_capacity);}    
    bool                verify_angle(float, float); // verify if vehicle's input angle is valid in roundabout
    void                find_ra_angle_unit(float, float); // use radius and safety velocity to compute angle unit
    void                Roundabout_information();
    void                current_situation(vector<Vehicle*>& ,vector<Vehicle*>&); // see the current situation when scheduling
    bool                check_intersection(float); 
    bool                check_conflict(int, vector<Vehicle*>&, vector<Vehicle*>&); 

    // schedule //
    void                greedy_without_safetymargin();

    // output the final solution //
    vector <vector <int> >  output_chart; // store the output
    void                output_solution();

    // vehicle variables //
    vector<Vehicle*>     v_total; // store each vehicle's properties

    
    // roundabout information //
    float               ra_time_unit; // unit : s
    float               ra_angle_unit; // unit : degree
    float               ra_radius;
    float               ra_safety_velocity;
    float               ra_safety_margin; // 小型車至少要保持「車速/2」距離(單位：公尺)；大型車至少要保持「車速-20」距離(單位：公尺)
    int                 ra_max_capacity;
    vector<float>       ra_valid_source_angle; // 0 <= angle < 2*pi
    vector<float>       ra_valid_destination_angle; // Si <= Di < Si + 2*pi ; D_i > 0

    void                reset();
    
};

#endif