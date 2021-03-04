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
#include "vehicle.h"

using namespace std;

//----------------------------------------------------------------------------------------
//    Function
//----------------------------------------------------------------------------------------
class Manager
{
  public:
    /* Default constructor */
    Manager() {};
    /* Define variables */
    int                 num_v_in_ra; // number of vehicles in the roundabout now
    vector<Vehicle>     total_v_info; // store each vehicle's properties
    /* Define function: read_file() */
    bool                check_vehicle_angle();
    void                read_vehicle();
    void                read_ra_info();
    /* Define function: schedule() */
    bool                check_constraint();
    void                do_scheduling();
    /* Define function: output_file() */
    void                do_output_file();
};

#endif