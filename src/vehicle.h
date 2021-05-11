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
  typedef pair<double, double> my_pair;
    public:
        /* Constructor */
        Vehicle(int v_id, double eat, double sa, double da, double vel)
        : id(v_id), earliest_arrival_time(eat), source_angle(sa), destination_angle(da), velocity(vel) {
          status = WAIT;
          now_angle = -1;
          angle_unit = 0; // unit: rad = 57.3 degree
          initial_priority = da-sa; // larger angle, higher priority
        };
        /* Vehicle status[ WAIT / IN / OUT ] */
        Vehicle_status              status;
        /* Vehicle variable, need to set value by other functions */
        int                         id;
        int                         source_intersection_id;
        int                         destination_intersection_id;
        double                       earliest_arrival_time;
        double                       source_angle; // 弧度
        double                       destination_angle; // 弧度
        double                       velocity;
        double                       initial_priority;
        /* output answer type, <t_1, angle_1>, <t_2, angle_2>, ... > */
        vector<my_pair>             position;

        double                       now_angle; 
        double                       angle_unit; 
        double                       safety_margin;
};

#endif
