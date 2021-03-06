/*****************************************************************************************
  FileName     [ ra_mgr_1.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Define basic variables for roundabout manager for case 1 in schema ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _RA_MGR_1_H
#define _RA_MGR_1_H

#include <vector>
#include <cstring>
#include <iostream>
#include <utility>
#include <cmath>
#include <map>
#include <climits>
#include "vehicle.h"

using namespace std;

class ra_mgr;
extern ra_mgr* raMgr;

// define the value of PI //
#define PI       3.14159265357
#define DELTA    1e-6   
#define TIMEUNIT 1e-3


//----------------------------------------------------------------------------------------
//    Function
//----------------------------------------------------------------------------------------
class ra_mgr
{
  // typedef map<const int, Vehicle*>   VehicleMap;

  public:
    ra_mgr();
    virtual ~ra_mgr() {}

    // read info about roundabout and vehicle -> call in main //
    bool                read_vehicle(const string&);
    bool                read_ra_info(const string&);

    // utility -> call in schedule  //
    bool                verify_angle(double, double); // verify if vehicle's input angle is valid in roundabout
    void                find_ra_angle_unit(double, double); // use radius and safety velocity to compute angle unit
    double              degree_to_rad(double degree) { return (degree*PI/180); }
    double              rad_to_degree(double rad) { return (rad*180/PI); }

    // information for check //
    void                Roundabout_information();
    void                Vehicle_information();

    // solution //
    short               _case; // case index
    void                line_trivial_solution_case_1();
    void                line_trivial_solution_case_2();
    void                line_trivial_solution_case_3();
    void                constant_velocity_skyline_solution_case_4();
    void                acceleration_solution_case_5();

    // newly add in skyline //
    vector< DLnode* >       _raSourceAngleList; 
    map<int, int>           _sourceAngletoId;
    map<int, int>           _destAngletoId;
    vector<DLnode*>         _upSkyline;
    vector<DLnode*>         _downSkyline;
    vector<DLnode*>         _skyline;
    vector<DLnode*>         _upperBoundSkyline;
    vector<DLnode*>         _lowerBoundSkyline;
    // new added
    map<string, Vehicle*>   _vId2VehicleMap;

    void                insertToEntry();
    void                updatePosition(Vehicle*);
    void                computeUDSkyline();
    void                computeSkyline();
    bool                canPlaceBetweenTwoSkyline(const int, const int);
    bool                checkIfBetweenUDSkyline(const int, const int);
    void                clearSkyline(vector<DLnode*>&);
    void                initGlobalVariables();

    vector<DLnode*>     answerList;
    // new added (Hugo)
    vector<Vehicle*>    sourceLatestVehicle;

    // newly add in segment implimentation of skyline;
    bool LinesConflicted(DLnode*, DLnode*);

    // output the final solution //
    // output format: v1 t1 p1 t2 p2 t3 p3 ....
    //                v2 t1 p1 t2 p2 t3 p3
    vector <vector < pair<int, int> > >  output_chart; // store the output
    void                                 output_solution(const string&);

    // vehicle variables //
    vector<Vehicle*>     v_total; // store each vehicle's properties, sort by eat
    // vector<Vehicle*>     wait_list; // sort by eat
    vector<double>       in_ra_time; // time that a car needs to go through the ra
    vector<double>       real_enter_time; // real time that a car enters the ra

    // const double         v_normal = 10;
    
    // roundabout information //
    double               ra_radius;
    double               ra_upper_velocity; // (v_max)
    double               ra_lower_velocity; // (v_min)
    double               ra_safety_margin; // 小型車至少要保持「車速/2」距離(單位：公尺)；大型車至少要保持「車速-20」距離(單位：公尺)
    int                  ra_max_capacity;
    int                  sa_size;
    vector<double>       ra_valid_source_angle; // 0 <= angle < 2*pi
    vector<double>       ra_valid_destination_angle; // Si <= Di < Si + 2*pi ; D_i > 0
    
    void                reset();

    // Debug
    void                  printSkyline(vector<DLnode*>&);
    
};

#endif
