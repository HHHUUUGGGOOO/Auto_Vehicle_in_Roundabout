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
#include "vehicle.h"
#include "ra_info.h"

using namespace std;

class ra_mgr;
extern ra_mgr* raMgr;



//----------------------------------------------------------------------------------------
//    Function
//----------------------------------------------------------------------------------------
class ra_mgr
{
  public:
    /* Default constructor */
    ra_mgr() {}
    virtual ~ra_mgr() {}

    /* Define function: read_file() */
    bool                read_vehicle(const string&);
    bool                read_ra_info(const string&);

    // utility //
    bool                verify_capacity();
    bool                verify_angle(float, float);

    /* Define function: schedule() */
    void                do_scheduling();
    void                greedy_without_safetymargin();

    /* Define function: output_file() */
    void                do_output_file();

    /* Define variables */
    int                 num_v_in_ra; // number of vehicles in the roundabout now
    vector<Vehicle>     total_v_info; // store each vehicle's properties
    
    // roundabout information //
    float           radius;
    float           safety_velocity;
    float           safety_margin;
    int             max_capacity;

    vector<float>   valid_source_angle; // 0 <= angle < 2*pi
    vector<float>   valid_destination_angle; // 0 <= angle < 2*pi
    
};

#endif