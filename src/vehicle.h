/*****************************************************************************************
  FileName     [ vehicle.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Define basic variables for vehicle property ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _VEHICLE_H
#define _VEHICLE_H

#include <vector>
#include <string>
#include <iostream>

using namespace std;

//----------------------------------------------------------------------------------------
//    Declaration
//----------------------------------------------------------------------------------------
enum Vehicle_status 
{ 
  WAIT = 0,
  IN = 1,
  OUT = 2 
};

class Vehicle
{
  typedef vector< pair<float, float> > my_pair;
    public:
        /* Default constructor */
        Vehicle(int v_id, float eat, float sa, float da)
        : id(v_id), earliest_arrival_time(eat), source_angle(sa), destination_angle(da) {
          priority = da-sa;
          status = WAIT;
        };
        /* Vehicle status, [ WAIT / IN / OUT ] */
        Vehicle_status              status;
        /* Roundabout variable, need to set value by other functions */
        int                         id;
        float                       earliest_arrival_time;
        float                       source_angle;
        float                       destination_angle;
        float                       priority;
        /* output answer type, < <angle_1, t_1>, <angle_2, t_2>, ... > */
        my_pair                     position;
};

#endif