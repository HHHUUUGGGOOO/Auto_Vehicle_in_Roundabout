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
  DLnode(int id, double startTime, double endTime = -1, double startAngle = -1, double endAngle = -1, bool start = false, bool exit = false):
    _id(id), _startTime(startTime), _endTime(endTime), _startAngle(startAngle), _endAngle(endAngle), _next(this), _prev(this), _front(this), _behind(this), is_start(start), is_exit(exit){}

  ~DLnode() {}

    //place function
    /* 
                    next
                     | 
      NOTE: behind - o - front
                     |
                    prev
    */
    void placeNextTo(DLnode* node) 
    {
      this->setPrev(node);
      this->setNext(node->getNext());
      node->getNext()->setPrev(this);
      node->setNext(this);
    }
    void placePreviousTo(DLnode* node) 
    {
      this->setNext(node);
      this->setPrev(node->getPrev());
      node->getPrev()->setNext(this);
      node->setPrev(this);
    }
    void placeInFrontOf(DLnode* node) 
    {
      this->setBehind(node);
      this->setFront(node->getFront());
      node->getFront()->setBehind(this);
      node->setFront(this);
    }
    void placeBehindOf(DLnode* node) 
    {
      this->setFront(node);
      this->setBehind(node->getBehind());
      node->getBehind()->setFront(this);
      node->setBehind(this);
    }    
    //set function
    void setNext(DLnode* next) {_next = next;}
    void setPrev(DLnode* prev) {_prev = prev;}
    void setFront(DLnode* front) {_front = front;}
    void setBehind(DLnode* behind) {_behind = behind;}

    //get function
    double getStartTime() {return _startTime;}
    double getEndTime() {return _endTime;}
    double getStartAngle() {return _startAngle;} //angle at start time
    double getEndAngle() {return _endAngle;} //angle at end time
    int getId() {return _id;}
    bool IsExit() {return is_exit;}
    bool IsStart() {return is_start;}
    DLnode* getNext() {return _next;}
    DLnode* getPrev() {return _prev;}
    DLnode* getFront() {return _front;}
    DLnode* getBehind() {return _behind;}

private:
    DLnode* _next;
    DLnode* _prev;
    DLnode* _front;
    DLnode* _behind;
    int _id;
    double _startTime;
    double _endTime;
    double _startAngle;
    double _endAngle;
    bool is_exit;
    bool is_start;
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
