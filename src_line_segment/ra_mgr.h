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
#include "vehicle.h"

using namespace std;

class ra_mgr;
extern ra_mgr* raMgr;

// define the value of PI //
#define PI      3.14159265357


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
    // bool                check_capacity() {return (v_in_ra_now >= ra_max_capacity);}    
    bool                verify_angle(double, double); // verify if vehicle's input angle is valid in roundabout
    void                find_ra_angle_unit(double, double); // use radius and safety velocity to compute angle unit
    void                current_situation(vector<Vehicle*>& ,vector<Vehicle*>&); // see the current situation when scheduling
    bool                check_intersection(double); // now_angle and next_angle 
    bool                check_conflict(int, vector<Vehicle*>&, vector<Vehicle*>&); 
    bool                check_conflict_by_wait(Vehicle const *, vector<Vehicle*>&);
    double               degree_to_rad(double degree) { return (degree*PI/180); }
    double               rad_to_degree(double rad) { return (rad*180/PI); }
    double               v_min_angle_unit(double angle) { return ((ceil(angle/ra_angle_unit))*ra_angle_unit); }


    // information for check //
    void                Roundabout_information();
    void                Vehicle_information();

    // solution //
    void                line_trivial_solution_case_1();
    void                line_trivial_solution_case_2();
    void                line_trivial_solution_case_3();
    void                skyline_solution_case_2();

    // newly add in skyline //
    vector< DLnode* > _raSourceAngleList;
    map<int, int> _sourceAngletoId;
    map<int, int> _destAngletoId;
    DLnode* _upSkyline;
    DLnode* _downSkyline;
    DLnode* _skyline;

    void insertToEntry(vector<DLnode*> & );
    void updatePosition(Vehicle* , const vector<DLnode*> &);
    void computeUDSkyline(const vector<DLnode*> &);
    void computeSkyline();
    bool canPlaceBetweenTwoSkyline(const vector<DLnode*> &, const double, const int);
    DLnode* clearSkyline(DLnode*);

    // newly add in segment implimentation of skyline;
    bool LinesConflicted(DLnode*, DLnode*);

    //map<int, int> vehicletoId;


    // output the final solution //
    // output format: v1 t1 p1 t2 p2 t3 p3 ....
    //                v2 t1 p1 t2 p2 t3 p3
    vector <vector < pair<int, int> > >  output_chart; // store the output
    void                                 output_solution(const string&);

    // vehicle variables //
    vector<Vehicle*>     v_total; // store each vehicle's properties, sort by Vehicle_ID
    vector<Vehicle*>     wait_list; // sort by eat
    vector<Vehicle*>     in_list;
    vector<double>       in_ra_time; // time that a car needs to go through the ra
    vector<double>       real_enter_time; // real time that a car enters the ra
    
    // roundabout information //
    // string              ra_purpose;
    double               ra_time_unit; // unit : s
    double               ra_angle_unit; // unit : degree
    double               ra_radius;
    double               ra_safety_velocity;
    double               ra_safety_margin; // 小型車至少要保持「車速/2」距離(單位：公尺)；大型車至少要保持「車速-20」距離(單位：公尺)
    int                  ra_max_capacity;
    vector<double>       ra_valid_source_angle; // 0 <= angle < 2*pi
    vector<double>       ra_valid_destination_angle; // Si <= Di < Si + 2*pi ; D_i > 0
    
    void                reset();

    // Debug
    void                  printSkyline(DLnode*);
    
};

#endif
