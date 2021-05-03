/*****************************************************************************************
  FileName     [ roundabout.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Header file of roundabout.cpp ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _ROUNDABOUT_H_
#define _ROUNDABOUT_H_

#include <string>

using namespace std;

//----------------------------------------------------------------------------------------
//    Declaration
//----------------------------------------------------------------------------------------
class Roundabout
{
    public:
        /* Default constructor */
        Roundabout() : radius(0.0), safety_velocity(0.0), safety_margin(0.0), max_capacity(0) {}; 
        /* Get vehicle Property */
        Roundabout(float r_id, float r_sv, float r_sm, int r_mc)
          : radius(r_id), safety_velocity(r_sv), safety_margin(r_sm), max_capacity(r_mc)
        {}


    private:
        /* Roundabout Property */
        float               radius; //unit?
        float               safety_velocity;
        float               safety_margin;
        int                 max_capacity;
        vector<float>       valid_source_angle; //optional
        vector<float>       valid_des_angle;  //optional                          
         
};

#endif