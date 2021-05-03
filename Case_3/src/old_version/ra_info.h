/*****************************************************************************************
  FileName     [ ra_info.h ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ Define basic variables for roundabout information ]
  Author       [ Yen-Yu, Chen & Hugo, Chen ]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
*****************************************************************************************/
//----------------------------------------------------------------------------------------
//    Include 
//----------------------------------------------------------------------------------------
#ifndef _RA_INFO_H_
#define _RA_INFO_H_

#include <vector>
#include <string>

using namespace std;

//----------------------------------------------------------------------------------------
//    Declaration
//----------------------------------------------------------------------------------------
class Roundabout
{
    public:
        /* Default constructor */
        Roundabout(): radius(10.0), safety_velocity(1.0), safety_margin(1.0), max_capacity(10) {
          for (int i = 0 ; i < 4; i++)
          {
            valid_source_angle.push_back(i*90);
            valid_destination_angle.push_back(i*90);
            valid_destination_angle.push_back((i+4)*90);
          }
        };

        /* Roundabout variables, need to default a value in the constructor */
        float           radius;
        float           safety_velocity;
        float           safety_margin;
        int             max_capacity;

        /* Roundabout variables, 0 <= angle < 2*pi */
        vector<float>   valid_source_angle; 
        vector<float>   valid_destination_angle; 
};

#endif