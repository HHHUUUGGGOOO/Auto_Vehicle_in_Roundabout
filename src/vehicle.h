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

using namespace std;

//----------------------------------------------------------------------------------------
//    Declaration
//----------------------------------------------------------------------------------------
enum class Vehicle_status { WAIT, IN, OUT };

class Vehicle
{
    public:
        /* Default constructor */
        Vehicle() {};
        /* Vehicle status, [ WAIT: 0 / IN: 1 / OUT: 2 ] */
        int                         in_roundabout;
        /* Roundabout variable, need to set value by other functions */
        int                         vehicle_id;
        float                       earliest_arrival_time;
        float                       source_angle;
        float                       destination_angle;
        float                       priority;
        /* output answer type, < <angle_1, t_1>, <angle_2, t_2>, ... > */
        vector<vector<float>>       position;
};

#endif