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
                            Vehicle() {}; // Default constructor
        /* Get vehicle Property, initialize with custom dimensions */
        Vehicle(int v_id, int v_priority, int v_depart_time, int v_route_id, string v_color)
          : vehicle_id(v_id), vehicle_priority(v_priority), depart_time(v_depart_time), route_id(v_route_id), vehicle_color(v_color)
        {}

    private:
        /* Vehicle Property */
        int                 vehicle_id { 0 };
        int                 vehicle_priority { 0 };
        int                 depart_time { 0 };
        int                 route_id { 0 };
        string              vehicle_color { "" }; // optional property
};

#endif