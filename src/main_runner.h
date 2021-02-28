/*****************************************************************************************
  FileName     [ main_runner.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Header file of main_runner.cpp ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _MAIN_RUNNER_H_
#define _MAIN_RUNNER_H_

#include <string>

using namespace std;

//----------------------------------------------------------------------------------------
//    Declaration
//----------------------------------------------------------------------------------------
class Vehicle
{
    public:
        /* Default constructor */
        Vehicle() : vehicle_id(0), vehicle_priority(0), depart_time(0.0), route_id(0), vehicle_color("") {}; 
        /* Get vehicle Property */
        Vehicle(int v_id, int v_priority, float v_depart_time, int v_route_id, string v_color)
          : vehicle_id(v_id), vehicle_priority(v_priority), depart_time(v_depart_time), route_id(v_route_id), vehicle_color(v_color)
        {}

    private:
        /* Vehicle Property */
        int                 vehicle_id;
        int                 vehicle_priority;
        float               depart_time;
        int                 route_id;
        string              vehicle_color; // optional property
};

#endif