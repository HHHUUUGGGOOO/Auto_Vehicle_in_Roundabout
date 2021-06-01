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
        cerr << endl;
        cerr << "Vehicle id: " <<  wait_list[i]->id << endl;
        enterAngleId = _sourceAngletoId[int(wait_list[i]->source_angle)];
        double da = (wait_list[i]->destination_angle > 360)? (wait_list[i]->destination_angle-360):wait_list[i]->destination_angle;
        exitAngleId = _destAngletoId[int(da)];

        // compute answerList //
        j = enterAngleId;
        t2 = wait_list[i]->earliest_arrival_time;
        while(j != exitAngleId)
        {
            angle1 = degree_to_rad(ra_valid_source_angle[j]);
            angle2 = (j+1 == sa_size)? degree_to_rad(ra_valid_source_angle[0]): degree_to_rad(ra_valid_source_angle[j+1]);
            if (angle2 < angle1) angle2 += 2*PI;
            t1 = t2;
            t2 = t1 + ra_radius*(angle2-angle1)/wait_list[i]->velocity;
            answerList[j] = new DLnode(wait_list[i]->id,t1, t2, angle1);
            cerr << "t1: " << t1 << " t2: " << t2 << " angle: " << angle1 << endl;

            j++;
            if (j == sa_size) j = 0;
        }
        answerList[j] = new DLnode(wait_list[i]->id, t2, -1, degree_to_rad(ra_valid_source_angle[j])); //dest // actually need to consider destination angle QQ
        cerr << "Out: t2: " << t2 << " angle: " << degree_to_rad(ra_valid_source_angle[j]) << endl;
        
        // compute upward and downward skyline //
        computeUDSkyline(answerList);

        // TODO: check if can fit between _downSkyline and _upSkyline else put based on _skyline
        // if can -> insert
        double safety_time_interval = ra_safety_margin / wait_list[i]->velocity;
        double temp_t2;
        if( !canPlaceBetweenTwoSkyline(answerList, safety_time_interval)){
            // if can't -> update answerList based on _skyline and insert
            printf("Vehicle %d can't Place Between Up and Down skyline\n", wait_list[i]->id);
            DLnode * node = _skyline;
            for (j = 0; j < enterAngleId; j++)
            {
                node = node->getFront();
            } // j = enterId

            // decide t2
            t2 = max(node->getT1()+safety_time_interval, wait_list[i]->earliest_arrival_time); // start time
            angle2 = node->getAngle();
            for (j = enterAngleId; j != exitAngleId; j = (j+1)%sa_size, node = node->getFront())
            {
                angle1 = node->getAngle();
                if (angle1 < angle2) angle1 += 2*PI;
                t1 = node->getT1()+safety_time_interval;
                temp_t2 = t1-ra_radius*(angle1-angle2)/wait_list[i]->velocity;
                if (temp_t2 > t2) t2 = temp_t2;
            }

            for (j = enterAngleId; j != exitAngleId; j = (j+1)%sa_size)
            {
                angle1 = degree_to_rad(ra_valid_source_angle[j]);
                angle2 = (j+1 == sa_size)? degree_to_rad(ra_valid_source_angle[0]) : degree_to_rad(ra_valid_source_angle[j+1]);
                if(angle2 < angle1) angle2 += 2*PI;
                t1 = t2;
                t2 = t1 + ra_radius*(angle2-angle1)/wait_list[i]->velocity; // May appears bug when velocity is not fixed.

                answerList[j] = new DLnode(wait_list[i]->id, t1, t2, angle1);
            }
            answerList[j] = new DLnode(wait_list[i]->id, t2, -1, degree_to_rad(ra_valid_source_angle[j]));
            
        }

        for (j = enterAngleId; j != exitAngleId; j = (j+1)%sa_size)
        {
            int nextJ = (j+1)%sa_size;
            answerList[j]->placeBehindOf(answerList[nextJ]);
        }

        for (j = enterAngleId; j != exitAngleId; j = (j+1)%sa_size){
            int nextJ = (j+1)%sa_size;
            answerList[j]->placeBehindOf(answerList[nextJ]);
        }

        updatePosition(wait_list[i], answerList);
        insertToEntry(answerList); // insert to _raSourceAngleList ans clear answerList

        // update _skyline //
        computeSkyline();
        //printSkyline(_skyline);
    }
    //for(int i = 0; i < _raSourceAngleList.size(); i++)
    //    printSkyline(_raSourceAngleList[i]);
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
            {
                _raSourceAngleList[i] = answerList[i];
            }
            else
            {
                if (node->getT1() > answerList[i]->getT1()) // first
                {
                    answerList[i]->placePreviousTo(node);
                    _raSourceAngleList[i] = answerList[i];
                }
                else
                {
                    while (node->getNext() != _raSourceAngleList[i] && node->getNext()->getT1() < answerList[i]->getT1())
                    {
                        node = node->getNext();
                    }
                    answerList[i]->placePreviousTo(node);
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
    // printf("\nUpdate position\n");
    // store answer to every vehicle //
    int i = _sourceAngletoId[int(v->source_angle)];
    double da = (v->destination_angle > 360)? (v->destination_angle-360):(v->destination_angle);
    int j = _destAngletoId[int(da)];
    while (i != j)
    {
        v->position.push_back(make_pair(answerList[i]->getT1(), rad_to_degree(answerList[i]->getAngle())));
        i++;
        if (i == ra_valid_source_angle.size()) i = 0;
    }
    v->position.push_back(make_pair(answerList[j]->getT1(), rad_to_degree(answerList[j]->getAngle())));

    // printf("\nEnd update position\n");
}

void
ra_mgr::computeUDSkyline(const vector<DLnode*> & answerList)
{
    // skyline must be new node different from _raSourceAngleList
    // skyline use cycle list for easy implement purpose

    // printf("\nCompute UD skyline\n");

    // clear skyline //
    _upSkyline = clearSkyline(_upSkyline);
    _downSkyline = clearSkyline(_downSkyline);

    // printf("\nEnd free\n");

    // construct skyline //
    int i;
    DLnode* nodeU;
    DLnode* nodeD;
    for (i = 0; i < _raSourceAngleList.size(); i++)
    {
        // find up and down //
        if (answerList[i] != NULL && _raSourceAngleList[i] != NULL)
        {
            if (_raSourceAngleList[i]->getT1() > answerList[i]->getT1())
            {
                nodeD = new DLnode(-1, INT_MIN, INT_MIN, degree_to_rad(ra_valid_source_angle[i]));
                nodeU = new DLnode(-1, _raSourceAngleList[i]->getT1(), _raSourceAngleList[i]->getT2(), _raSourceAngleList[i]->getAngle());
            }
            else
            {
                nodeD = _raSourceAngleList[i];
                while(nodeD->getNext() != _raSourceAngleList[i] && nodeD->getNext()->getT1() < answerList[i]->getT1())
                {
                    nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[i])
                    nodeU = new DLnode(-1, INT_MAX, INT_MAX, degree_to_rad(ra_valid_source_angle[i]));
                else
                    nodeU = new DLnode(-1, nodeU->getT1(), nodeU->getT2(), nodeU->getAngle());
                nodeD = new DLnode(-1, nodeD->getT1(), nodeD->getT2(), nodeD->getAngle());   
            }
        }
        else
        {
            nodeU = new DLnode(-1, INT_MAX, INT_MAX, degree_to_rad(ra_valid_source_angle[i]));
            nodeD = new DLnode(-1, INT_MIN, INT_MIN, degree_to_rad(ra_valid_source_angle[i]));
        }

        // insert to the last //
        // printf("\nInsert to the last\n");
        if (_upSkyline == NULL) { _upSkyline = nodeU; }
        else { nodeU->placeBehindOf(_upSkyline); }

        if (_downSkyline == NULL) { _downSkyline = nodeD; }
        else { nodeD->placeBehindOf(_downSkyline); }
    }

    // printf("End Compute UD skyline\n");
}

void
ra_mgr::computeSkyline()
{
    // printf("\nCompute skyline\n");
    // clear skyline //
    _skyline = clearSkyline(_skyline);

    int i;
    DLnode* node;
    for (i = 0; i < _raSourceAngleList.size(); i++)
    {
        // find overall skyline (each entry's prev) //
        if (_raSourceAngleList[i] == NULL)
        {
            node = new DLnode(-1, INT_MIN, INT_MIN, degree_to_rad(ra_valid_source_angle[i]));
        }
        else
        {
            //printSkyline(_raSourceAngleList[i]);
            //printf("%lf\n",  _raSourceAngleList[i]->getPrev()->getT1());
            node = _raSourceAngleList[i]->getPrev();
            node = new DLnode(-1, node->getT1(), node->getT2(), node->getAngle());
        }

        // insert to the last //
        if (_skyline == NULL)
        {
            _skyline = node;
        }
        else
        {
            node->placeBehindOf(_skyline);
        }
    }

    // printf("End Compute UD skyline\n");
}


bool
ra_mgr::canPlaceBetweenTwoSkyline(const vector<DLnode*> & answerList, const double time_interval)
{
    // printf("\ncanPlaceBetweenTwoSkyline\n");
    int i, sa_size;
    DLnode* upNode = _upSkyline, *downNode = _downSkyline;
    for (i = 0, sa_size = _raSourceAngleList.size(); i < sa_size; i++)
    {
        if (answerList[i] != NULL)
        {
            if(upNode->getT1() - answerList[i]->getT1() < time_interval) return false;
            if(answerList[i]->getT1() - downNode->getT1() < time_interval) return false;
        }
        upNode = upNode->getFront();
        downNode = downNode->getFront();
    }
    return true;
}

void ra_mgr::printSkyline(DLnode *node){
    DLnode *tmp = node;
    DLnode *start = node;
    while(tmp->getFront() != start){
        printf("%lf ", tmp->getT1());
        tmp = tmp->getFront();
    }
    printf("%lf\n", tmp->getT1());
}

DLnode* ra_mgr::clearSkyline(DLnode *skyline){
    if (skyline!= NULL) // for the first time
    {
        DLnode *curNode = skyline->getFront();
        DLnode *nextNode = NULL;

        while(curNode != skyline)
        {
            nextNode = curNode->getFront();
            free(curNode);
            curNode = nextNode;
        }
        free(skyline);
    }
    return NULL;
}
