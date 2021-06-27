#include "checker.h"

inline double velocity(Segment* const seg, double _raRadius)
{
    if (seg->angle1()>seg->angle2())
        return(_raRadius*((seg->angle2()+360.0-seg->angle1())*(PI/180))/(seg->t2()-seg->t1()));
    else 
        return(_raRadius*((seg->angle2()-seg->angle1())*(PI/180))/(seg->t2()-seg->t1()));
}

bool sortByAngle1(Segment* a, Segment* b)
{
    return (a->angle1() < b->angle1());
}

void Checker::check(const string& vFile, const string& raFile, const string& outFile)
{
    // read roundabout information //
    if (!read_raFile(raFile)) return;

    // read output //
    // check velocity //
    if (!read_outFile(outFile)) return;

    // read vehicle information //
    // check source and destination angle //
    if (!read_vFile(vFile)) return;

    // check safety margin //
    printf("\ncheck safety margin\n");
    _timeSeglist.resize(_timelist.size());
    
    for (auto i: _vSeglist)
        for(auto seg: i)
            for (int t = 0; t < _timelist.size(); t++)
            {
                if (((seg->t1()-error/100) < (_timelist[t]/10000.0)) && ((seg->t2()+error/100) > (_timelist[t]/10000.0)))
                {
                    _timeSeglist[t].push_back(seg);
                    break;
                }
            }        

    double anglePrev, angleNext, timeUnit, angleUnit;
    for (int i = 0; i < _timeSeglist.size(); i++)
    {
        if (_timeSeglist[i].size() < 2) continue;
        sort(_timeSeglist[i].begin(), _timeSeglist[i].end(), sortByAngle1);
        for (int j = 0; j < _timeSeglist[i].size(); j++)
        {
            for (int k = j+1; k < _timeSeglist[i].size(); k++)
            {
                if (_timeSeglist[i][j]->status() == EXIT && _timeSeglist[i][k]->status() == IN)
                    continue;
                
                angleUnit = _timeSeglist[i][j]->angle2() - _timeSeglist[i][j]->angle1();
                if (angleUnit < 0) angleUnit += 360.0;
                angleUnit /= (_timeSeglist[i][j]->t2() - _timeSeglist[i][j]->t1());
                anglePrev = _timeSeglist[i][j]->angle1() + angleUnit*((_timelist[i]/10000.0)-_timeSeglist[i][j]->t1());
                if (anglePrev > 360.0) anglePrev-=360.0;

                angleUnit = _timeSeglist[i][k]->angle2() - _timeSeglist[i][k]->angle1();
                if (angleUnit < 0) angleUnit += 360.0;
                angleUnit /= (_timeSeglist[i][k]->t2() - _timeSeglist[i][k]->t1());
                angleNext = _timeSeglist[i][k]->angle1() + angleUnit*((_timelist[i]/10000.0)-_timeSeglist[i][k]->t1());
                if (angleNext > 360.0) angleNext-=360.0;

                if ((_raRadius*(abs(angleNext-anglePrev)*(PI/180.0)) < _safetyMargin-error)
                    || (_raRadius*(abs(angleNext-anglePrev+360.0)*(PI/180.0)) < _safetyMargin-error))
                {
                    cerr << "wrong" << endl;
                    cerr << _raRadius*(abs(angleNext-anglePrev)*(PI/180.0)) << endl;
                    cerr << _safetyMargin << endl; 
                    cerr << _timelist[i]/10000.0 << endl;
                    cerr << _timeSeglist[i][j]->id() << endl;
                    cerr << anglePrev << endl;
                    cerr << _timeSeglist[i][k]->id() << endl;
                    cerr << angleNext << endl;
                    return;
                }
            }
                
        }
    }


    // output pass //
    cout << "---------------------" << endl;
    cout << "last time: " << _timelist[_timelist.size()-1]/10000.0 << endl;
    cout << "Verification Pass!! ＼（＾∀＾）X（＾∀＾）ノ" << endl;
    cout << "---------------------" << endl;
}

bool Checker::read_raFile(const string& raFile)
{
    printf("\nRead roundabout input file...(%s)\n", raFile.c_str());
    fstream fin(raFile.c_str());
    if (!fin)
    {
        cout << "File " << raFile << " could not be opened!!" << endl;
        return false;
    }

    fin >> _raRadius >> _lowerVelocity >> _upperVelocity >> _safetyMargin >> _maxCapacity;

    int num, i, tmpI;
    double tmp;
    fin >> num;
    for (i = 0; i < num; i++)
    {
        fin >> tmp;
        _entrylist.push_back(tmp);
        tmpI = round(tmp*10);
        _entryMap[tmpI] = i;
    }

    fin >> num;
    for (i = 0; i < num; i++)
    {
        fin >> tmp;
        _exitlist.push_back(tmp);
        tmpI = round(tmp*10);
        _exitMap[tmpI] = i;
    }

    fin.close();
    return true;
}

bool Checker::read_vFile(const string& vFile)
{
    printf("\nRead vehicle input file...(%s)\n", vFile.c_str());
    fstream fin(vFile.c_str());
    if (!fin)
    {
        cout << "File " << vFile << " could not be opened!!" << endl;
        return false;
    }

    int count = 0, id, size;
    double sourceAngle, destinationAngle, entryTime, InitialVel;
    string input;

    while(count < _vehicleNum)
    {
        getline(fin, input);
        istringstream tokens(input);

        tokens >> id >> entryTime >> sourceAngle >> destinationAngle >> InitialVel;
        size = _vSeglist[_id2index[id]].size();
        if (_vSeglist[_id2index[id]][0]->t1() < entryTime)
        {
            cout << "---------------------" << endl;
            cout << "vehicle id: "<< id << " enter early than schedule: " << _vSeglist[_id2index[id]][0]->t1() << "(s)" << endl;
            cout << "It's schedule entering time: " << entryTime << "(s) ( ´･ω･)" << endl;
            cout << "---------------------" << endl;
            return false;
        }
        else if ((_vSeglist[_id2index[id]][0]->angle1() < sourceAngle-error)
            || (_vSeglist[_id2index[id]][0]->angle1() > sourceAngle+error) )
        {
            cout << "---------------------" << endl;
            cout << "vehicle id: "<< id << " enter at wrong entry: " << _vSeglist[_id2index[id]][0]->angle1() << "(degree)" << endl;
            cout << "It's schedule entering entry: " << sourceAngle << "(degree) ( ´･ω･)" << endl;
            cout << "---------------------" << endl;
            return false;
        }
        else if ((_vSeglist[_id2index[id]][size-1]->angle2() < destinationAngle-error)
            || (_vSeglist[_id2index[id]][size-1]->angle2() > destinationAngle+error))
        {
            cout << "---------------------" << endl;
            cout << "vehicle id: "<< id << " enter at wrong exit: " << _vSeglist[_id2index[id]][size-1]->angle2() << "(degree)" << endl;
            cout << "It's schedule exiting entry: " << destinationAngle << "(degree) ( ´･ω･)" << endl;
            cout << "---------------------" << endl;
            return false;
        }
        count++;
    }
    return true;
}

bool Checker::read_outFile(const string& outFile)
{
    printf("\nRead output file...(%s)\n", outFile.c_str());
    fstream fin(outFile.c_str());
    if (!fin)
    {
        cout << "File " << outFile << " could not be opened!!" << endl;
        return false;
    }

    string input;
    int id, index=0, indextmp;
    double entryAngle, entryNextAngle, exitAngle, exitPrevAngle;
    double t1, angle1, t2, angle2, vel;
    Segment* segment;

    fin >> _vehicleNum;
    _vSeglist.resize(_vehicleNum);
    getline(fin, input); // remove dummy line
    while (index < _vehicleNum)
    {
        getline(fin, input);
        istringstream tokens(input);

        tokens >> id;
        _id2index[id] = index;
        tokens >> t1 >> angle1;

        if (_entryMap.find(round(angle1*10)) == _entryMap.end())
        {
            cout << "---------------------" << endl;
            cout << "vehicle id: " << id << " not enter in given entering entry!!(" << angle1 << ") ( ´･ω･)"<< endl;
            cout << "---------------------" << endl;
            return false;
        }
        indextmp = _entryMap[round(angle1*10)];
        entryAngle = _entrylist[indextmp];
        if (indextmp == _entrylist.size()-1)
            entryNextAngle = 360.0;
        else
            entryNextAngle = _entrylist[indextmp+1];

        while(tokens >> t2 >> angle2)
        {
            segment = new Segment(id, t1, t2, angle1, angle2);
            _vSeglist[index].push_back(segment);
            t1 = t2;
            angle1 = angle2;
        }

        if (_exitMap.find(round(angle1*10)) == _exitMap.end())
        {
            cout << "---------------------" << endl;
            cout << "vehicle id: " << id << " not exit in given exiting entry!!(" << angle1 << ") ( ´･ω･)"<< endl;
            cout << "---------------------" << endl;
            return false;
        }
        indextmp = _exitMap[round(angle1*10)];
        exitAngle = _exitlist[indextmp];
        if (exitAngle == 0)
        {
            exitAngle = 360.0;
            exitPrevAngle = _exitlist[_exitlist.size()-1];
        }
        else
            exitPrevAngle = _exitlist[indextmp-1];

        for (auto seg: _vSeglist[index])
        {
            // update status //
            if (seg->angle2() <= entryNextAngle && seg->angle2() > entryAngle)
                seg->setStatus(ENTRY);
            else if (seg->angle1() < exitAngle && seg->angle1() >= exitPrevAngle)
                seg->setStatus(EXIT);
            
            // check velocity //
            vel = velocity(seg, _raRadius);
            if ((vel > _upperVelocity+error) || (vel < _lowerVelocity-error))
            {
                cout << "---------------------" << endl;
                cout << "vehicle id: "<< seg->id() << " violate velocity constraint with velocity: " << vel << endl;
                cout << "under safety velocity range(" << _upperVelocity << "," << _lowerVelocity << ") ( ´･ω･)" << endl;
                cout << "---------------------" << endl;
                return false;
            }

            // construct timelist //
            // if (seg->id() == 1)
                // cerr << int(round(seg->t1()*10000.0)) << " " << int(round(seg->t2()*10000.0)) << endl;
            if (find(_timelist.begin(), _timelist.end(),int(round(seg->t1()*10000.0))) == _timelist.end())
            {
                // if (seg->id() == 1)
                //     cerr << "add t1" << endl;
                _timelist.push_back(round(seg->t1()*10000.0));
            }
            if (find(_timelist.begin(), _timelist.end(), int(round(seg->t2()*10000.0))) == _timelist.end())
            {
                // if (seg->id() == 1)
                //     cerr << "add t2" << endl;
                _timelist.push_back(round(seg->t2()*10000.0));
            }
        }
        index++;
    }
    sort(_timelist.begin(), _timelist.end());
    return true;
}

