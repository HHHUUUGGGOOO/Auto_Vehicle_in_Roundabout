#include <iostream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <vector>
#include <utility>
#include <map>
#include <cmath>
#include <algorithm>
#include <cmath>

using namespace std;

#define ENTRY 0
#define EXIT 1
#define IN 2

// define the value of PI //
#define PI      3.14159265357

class Segment
{
public:
    Segment(int id, double t1, double t2, double angle1, double angle2)
        :_id(id), _t1(t1), _t2(t2), _angle1(angle1), _angle2(angle2), _status(IN) {}

    // set //
    void setStatus(short s) { _status = s; }

    // get //
    int id() { return _id; }
    double t1() { return _t1; }
    double t2() { return _t2; }
    double angle1() { return _angle1; }
    double angle2() { return _angle2; }
    short status() { return _status; }

private:
    int _id;
    double _t1;
    double _t2;
    double _angle1;
    double _angle2;
    short _status;
};

class Checker
{
public:
    Checker() {}
    ~Checker () {}

    void check(const string&, const string&, const string&);

    // read //
    bool read_raFile(const string&);
    bool read_outFile(const string&);
    bool read_vFile(const string&);

private:
    // error // 
    double error = 0.01;

    // roundabout information //
    double _upperVelocity;
    double _lowerVelocity;
    double _raRadius;
    double _safetyMargin;
    double _maxCapacity;
    
    vector<double> _entrylist;
    map<int, int> _entryMap; // angle*10 to int key
    vector<double> _exitlist;
    map<int, int> _exitMap; // angle*10 to int key

    // vehicle information //
    vector<vector<Segment*> > _vSeglist;
    map<int, int> _id2index;
    int _vehicleNum;

    // time list // 
    vector<vector<Segment*> > _timeSeglist;
    vector<int> _timelist; // time*10 to int
};