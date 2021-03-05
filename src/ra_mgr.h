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
    bool                check_vehicle_angle();
    bool                read_vehicle(const string& infile);
    // bool                read_ra_info(const string& rafile);

    /* Define function: schedule() */
    bool                check_constraint();
    bool                do_scheduling();

    /* Define function: output_file() */
    void                do_output_file();

    /* Define variables */
    int                 num_v_in_ra; // number of vehicles in the roundabout now
    vector<Vehicle>     total_v_info; // store each vehicle's properties
    
};

#endif