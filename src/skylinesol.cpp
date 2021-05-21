/*****************************************************************************************
  FileName     [ skylinesol.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ based on skyline problem to solve ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
***************************************************************************************/

#include "vehicle.h"
#include "ra_mgr.h"
#include <climits>

void 
ra_mgr::skyline_solution_case_2()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }

    int i, j, v_size, sa_size;
    int upId, downId;
    double enterAngleId, exitAngleId, t1, t2, angle1, angle2;
    sa_size = ra_valid_source_angle.size();
    _raSourceAngleList.resize(sa_size);
    vector<DLnode*> answerList(sa_size); // temp answer list for one vehicle
    
    // assume same velocity // note: wait_list is sort by its time
    for (i = 0, v_size = wait_list.size(); i < v_size; i++)
    {
        cerr << "Vehicle id: " <<  wait_list[i]->id << endl;
        enterAngleId = _sourceAngletoId[int(wait_list[i]->source_angle)];
        exitAngleId = _destAngletoId[int(wait_list[i]->destination_angle)];

        // compute answerList //
        j = enterAngleId;
        t2 = wait_list[i]->earliest_arrival_time;
        while(j != exitAngleId)
        {
            angle1 = degree_to_rad(ra_valid_source_angle[j]);
            angle2 = (j+1 == sa_size)? degree_to_rad(ra_valid_source_angle[0]): degree_to_rad(ra_valid_source_angle[j+1]);
            if (angle2 < angle1) angle2 += 2*PI;
            t1 = t2;
            t2 = t1 + (angle2-angle1)/wait_list[i]->velocity;
            answerList[j] = new DLnode(wait_list[i]->id,t1, t2, angle1);

            j++;
            if (j == sa_size) j = 0;
        }
        answerList[j] = new DLnode(wait_list[i]->id, t2, -1, degree_to_rad(ra_valid_source_angle[j])); //dest // actually need to consider destination angle QQ

        // compute upward and downward skyline //
        computeSkyline(answerList);

        // TODO: check if can fit between _downSkyline and _upSkyline else put based on _skyline
        // if can -> insert
        updatePosition(wait_list[i], answerList);
        insertToEntry(answerList); // insert to _raSourceAngleList ans clear answerList
        // if can't -> update answerList based on _skyline and insert

        // TODO: check if need to update _skyline

    }
}

void 
ra_mgr::insertToEntry(vector<DLnode*> & answerList)
{
    int i, sa_size;
    DLnode* node;
    for (i = 0, sa_size = _raSourceAngleList.size(); i < sa_size; i++)
    {
        node = _raSourceAngleList[i];
        if (answerList[i] != NULL)
        {
            if (node == NULL)
                _raSourceAngleList[i] = answerList[i];
            else
            {
                if (node->getT1() > answerList[i]->getT1()) // first
                {
                    answerList[i]->setNext(node);
                    node->setPrev(answerList[i]);
                    _raSourceAngleList[i] = answerList[i];
                }
                else
                {
                    while (node->getNext() != NULL && node->getNext()->getT1() < answerList[i]->getT1())
                    {
                        node = node->getNext();
                    }
                    answerList[i]->setPrev(node);
                    answerList[i]->setNext(node->getNext());

                    node->setNext(answerList[i]);
                    if (node->getNext() != NULL) node->getNext()->setPrev(answerList[i]);
                }
            }
        }
        // clear answerList for next vehicle //
        answerList[i] = NULL;
    }
}

void
ra_mgr::updatePosition(Vehicle* v, const vector<DLnode*> & answerList)
{
    // store answer to every vehicle //
    int i = _sourceAngletoId[int(v->source_angle)];
    int j = _destAngletoId[int(v->destination_angle)];
    while (i != j)
    {
        v->position.push_back(make_pair(answerList[i]->getT1(), rad_to_degree(answerList[i]->getAngle())));
        i++;
        if (i == ra_valid_source_angle.size()) i = 0;
    }
}

void
ra_mgr::computeSkyline(const vector<DLnode*> & answerList)
{
    // skyline must be new node different from _raSourceAngleList
    // skyline use cycle list for easy implement purpose

    // clear skyline //
    DLnode* nodeU = _upSkyline;
    DLnode* nextU = NULL;
    DLnode* nodeD = _downSkyline;
    DLnode* nextD = NULL;

    while(nodeU != NULL || nodeD != NULL)
    {
        if (nodeU != NULL)
        {
            nextU = nodeU->getNext();
            free(nodeU);
            nodeU = nextU;
        }

        if (nodeD != NULL)
        {
            nextD = nodeD->getNext();
            free(nodeD);
            nodeD = nextD;
        }
    }

    // construct skyline //
    int i;
    for (i = 0; i < _raSourceAngleList.size(); i++)
    {
        // find up and down //
        if (answerList[i] != NULL && _raSourceAngleList[i] != NULL)
        {
            if (_raSourceAngleList[i]->getT1() > answerList[i]->getT1())
            {
                nodeD = new DLnode(-1, 0, 0, degree_to_rad(ra_valid_source_angle[i]));
                nodeU = new DLnode(-1, _raSourceAngleList[i]->getT1(), _raSourceAngleList[i]->getT2(), _raSourceAngleList[i]->getAngle());
            }
            else
            {
                nodeD = _raSourceAngleList[i];
                while(nodeD->getNext() != NULL && nodeD->getNext()->getT1() < answerList[i]->getT1())
                {
                    nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == NULL)
                    nodeU = new DLnode(-1, INT_MAX, INT_MAX, degree_to_rad(ra_valid_source_angle[i]));
                else
                    nodeU = new DLnode(-1, nodeU->getT1(), nodeU->getT2(), nodeU->getAngle());
                nodeD = new DLnode(-1, nodeD->getT1(), nodeD->getT2(), nodeD->getAngle());   
            }
        }
        else
        {
            nodeU = new DLnode(-1, INT_MAX, INT_MAX, degree_to_rad(ra_valid_source_angle[i]));
            nodeD = new DLnode(-1, 0, 0, degree_to_rad(ra_valid_source_angle[i]));
        }

        // insert to the last //
        if (_upSkyline == NULL)
        {
            _upSkyline = nodeU;
            _upSkyline->setPrev(_upSkyline);
            _upSkyline->setNext(_upSkyline);
        }
        else
        {   
            nodeU->setPrev(_upSkyline->getPrev());
            _upSkyline->getPrev()->setPrev(nodeU);
            nodeU->setNext(_upSkyline);
            _upSkyline->setPrev(nodeU);
        }

        if (_downSkyline == NULL)
        {
            _downSkyline = nodeD;
            _downSkyline->setPrev(_downSkyline);
            _downSkyline->setNext(_downSkyline);
        }
        else
        {   
            nodeD->setPrev(_downSkyline->getPrev());
            _downSkyline->getPrev()->setPrev(nodeD);
            nodeD->setNext(_downSkyline);
            _downSkyline->setPrev(nodeD);
        }
    }
}
