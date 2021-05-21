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

// to handle list in every source and destination angle //
class DLnode
{
public:
  DLnode(int id, double t1, double t2 = -1, double angle = -1):
    _id(id), _t1(t1), _t2(t2), _angle(angle), _next(NULL), _prev(NULL) {}
  ~DLnode() {}

    //set function
    void setNext(DLnode* next) {_next = next;}
    void setPrev(DLnode* prev) {_prev = prev;}

    //get function
    double getT1() {return _t1;}
    double getT2() {return _t2;}
    double getAngle() {return _angle;} //angle at t1
    int getId() {return _id;}
    DLnode* getNext() {return _next;}
    DLnode* getPrev() {return _prev;}

private:
    DLnode* _next;
    DLnode* _prev;
    int _id;
    double _t1;
    double _t2;
    double _angle;
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
