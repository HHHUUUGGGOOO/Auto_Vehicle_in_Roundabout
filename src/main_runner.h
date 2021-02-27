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
                            Vehicle() {}; // constructor
    private:
        /* Vehicle Property */
        int                 vehicle_id;
        int                 vehicle_priority;
        int                 deparrt_time;
        int                 route_id;
        string              vehicle_color;
};

#endif