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
#include <cmath>

void 
ra_mgr::skyline_solution_case_2()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }

    int sa_size = ra_valid_source_angle.size();
    int v_size = wait_list.size();
    _raSourceAngleList.resize(sa_size);
    vector<DLnode*> answerList(sa_size); // temp answer list for one vehicle,       answerList[0]: intersection_0 -> intersection_1, ... , answerList[sa_size-1] = intersection_{sa_size} -> intersection_0
    
    // assume same velocity // note: wait_list is sort by its time
    for (int current_v_id = 0; current_v_id < v_size; current_v_id++)
    {
        
        cerr << endl;
        cerr << "Vehicle id: " <<  wait_list[current_v_id]->id << endl;
        double  startTime, endTime, startAngle, endAngle;

        // TODO:
        // Below _sourceAngletoId and _destAngletoId will conflict if they are not the same;
        double sa = wait_list[current_v_id]->source_angle;
        int enterAngleId = _sourceAngletoId[int(sa)];
        double da = (wait_list[current_v_id]->destination_angle > 360)? (wait_list[current_v_id]->destination_angle-360):wait_list[current_v_id]->destination_angle;
        int exitAngleId = _destAngletoId[int(da)];

        // compute answerList //
        
        for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
        {
            startAngle = degree_to_rad(ra_valid_source_angle[currentAngleId]);
            int nextIntersectionId = (currentAngleId + 1 == sa_size)? 0:currentAngleId+1; 
            endAngle = degree_to_rad(ra_valid_source_angle[nextIntersectionId]);
            if (endAngle < startAngle) { endAngle += 2*PI; }
            startTime = (currentAngleId == enterAngleId)? wait_list[current_v_id]->earliest_arrival_time : endTime;
            endTime = startTime + ra_radius*(endAngle-startAngle)/wait_list[current_v_id]->velocity;
            answerList[currentAngleId] = new DLnode(wait_list[current_v_id]->id,startTime, endTime, startAngle, (currentAngleId == enterAngleId)/*start*/, (currentAngleId == exitAngleId)/*exit*/);
            cerr << "Start time: " << startTime << ", end time: " << endTime << ", start angle: " << startAngle << endl;
        }
        
        // compute upward and downward skyline //
        computeUDSkyline(answerList);

        // TODO: check if can fit between _downSkyline and _upSkyline else put based on _skyline
        // if can -> insert
        double safety_time_interval = ra_safety_margin / wait_list[current_v_id]->velocity;
        bool noAnswer = false;
        DLnode *nodeU, *nodeD;
        double timeUnit = 1e-3;
        while(!canPlaceBetweenTwoSkyline(answerList, safety_time_interval, enterAngleId)){
            
            nodeD = _raSourceAngleList[enterAngleId];
            if ((nodeD->getStartTime()-safety_time_interval) > answerList[enterAngleId]->getStartTime())
            {
                endTime = answerList[enterAngleId]->getStartTime() + timeUnit;
            }
            else
            {
                while(nodeD->getNext() != _raSourceAngleList[enterAngleId] && nodeD->getNext()->getStartTime() < answerList[enterAngleId]->getStartTime())
                {
                    nodeD = nodeD->getNext();
                    while (nodeD->IsExit() && nodeD->getNext() != _raSourceAngleList[enterAngleId])
                        nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[enterAngleId])
                {
                    noAnswer = true;
                    break;
                }
                else
                {
                    if (nodeU->getStartTime() > answerList[enterAngleId]->getStartTime() + timeUnit + safety_time_interval)
                        endTime = answerList[enterAngleId]->getStartTime() + timeUnit;
                    else endTime = nodeU->getStartTime() + safety_time_interval;
                }
            }

            // free answerlist //
            for (int j = 0; j < sa_size; j++)
            {
                if (answerList[j] != NULL)
                {
                    free(answerList[j]);
                }
            }


            // compute new answerlist and check if it can put between skyline //
            
            for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                startAngle = degree_to_rad(ra_valid_source_angle[currentAngleId]);
                endAngle = (currentAngleId+1 == sa_size)? degree_to_rad(ra_valid_source_angle[0]): degree_to_rad(ra_valid_source_angle[currentAngleId+1]);
                if (endAngle < startAngle) endAngle += 2*PI;
                startTime = endTime;
                endTime = startTime + ra_radius*(endAngle-startAngle)/wait_list[current_v_id]->velocity;
                answerList[currentAngleId] = new DLnode(wait_list[current_v_id]->id, startTime, endTime, startAngle);
                // cerr << "startTime: " << startTime << " endTime: " << endTime << " angle: " << startAngle << endl;
            }
            // cerr << "New out: endTime: " << endTime << " angle: " << degree_to_rad(ra_valid_source_angle[j]) << endl;
            computeUDSkyline(answerList);
        }
    
        if (noAnswer)
        {
            // update answerList based on _skyline and insert
            printf("Vehicle %d can't place between skyline\n", wait_list[current_v_id]->id);
            DLnode * node = _skyline;
            for (int j = 0; j < enterAngleId; j++)
            {
                node = node->getFront();
            } // node is the correspondant node on _skyline wrt. enterAngleId

            // decide endTime
            endTime = max(node->getStartTime()+safety_time_interval, wait_list[current_v_id]->earliest_arrival_time); // start time
            endAngle = node->getEndAngle();
            // Find end time at Intersection_enterAngleId

            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size, node = node->getFront())
            {
                startAngle = node->getStartAngle();
                if (startAngle < endAngle) startAngle += 2*PI;
                startTime = node->getStartTime()+safety_time_interval;
                double temp_endTime = startTime-ra_radius*(startAngle-endAngle)/wait_list[current_v_id]->velocity;
                if (temp_endTime > endTime) endTime = temp_endTime;
            }

            for (int j = enterAngleId; j != exitAngleId; j = (j+1)%sa_size)
            {
                startAngle = degree_to_rad(ra_valid_source_angle[j]);
                endAngle = (j+1 == sa_size)? degree_to_rad(ra_valid_source_angle[0]) : degree_to_rad(ra_valid_source_angle[j+1]);
                if(endAngle < startAngle) endAngle += 2*PI;
                startTime = endTime;
                endTime = startTime + ra_radius*(endAngle-startAngle)/wait_list[current_v_id]->velocity; // May appears bug when velocity is not fixed.

                answerList[j] = new DLnode(wait_list[current_v_id]->id, startTime, endTime, startAngle, (j == enterAngleId), (j==exitAngleId));
            }
            
        }


        for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
        {
            int nextAngleId = (currentAngleId+1)%sa_size;
            answerList[currentAngleId]->placeBehindOf(answerList[nextAngleId]);
        }

        updatePosition(wait_list[current_v_id], answerList);
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
    int sa_size = _raSourceAngleList.size();
    for (int currentAngleId = 0; currentAngleId < sa_size; currentAngleId++)
    {
        DLnode* node = _raSourceAngleList[currentAngleId];
        if (answerList[currentAngleId] != NULL)
        {
            if (node == NULL)
            {
                _raSourceAngleList[currentAngleId] = answerList[currentAngleId];
            }
            else
            {
                if (node->getStartTime() > answerList[currentAngleId]->getStartTime()) // first
                {
                    answerList[currentAngleId]->placePreviousTo(node);
                    _raSourceAngleList[currentAngleId] = answerList[currentAngleId];
                }
                else
                {
                    while (node->getNext() != _raSourceAngleList[currentAngleId] && node->getNext()->getStartTime() < answerList[currentAngleId]->getStartTime())
                    {
                        node = node->getNext();
                    }
                    answerList[currentAngleId]->placeNextTo(node);
                }
            }
        }
        // clear answerList for next vehicle //
        answerList[currentAngleId] = NULL;
    }
}

void
ra_mgr::updatePosition(Vehicle* v, const vector<DLnode*> & answerList)
{
    // printf("\nUpdate position\n");
    // store answer to every vehicle //
    double da = (v->destination_angle > 360)? (v->destination_angle-360):(v->destination_angle);
    int destinationAngleId = _destAngletoId[int(da)];
    int ra_valid_destination_size = ra_valid_destination_angle.size();
    for(int currentAngleId =  _sourceAngletoId[int(v->source_angle)]; currentAngleId != destinationAngleId; currentAngleId = (currentAngleId+1) % ra_valid_destination_size){

        v->position.push_back(make_pair(answerList[currentAngleId]->getStartTime(), rad_to_degree(answerList[currentAngleId]->getStartAngle())));
        
        int nextCurrentId = (currentAngleId+1)%ra_valid_destination_size;
        if(nextCurrentId == destinationAngleId)
        {
            v->position.push_back(make_pair(answerList[currentAngleId]->getEndTime(), rad_to_degree(answerList[currentAngleId]->getEndAngle())));
        }
    }
    /* Previous 
    int i = _sourceAngletoId[int(v->source_angle)];
    double da = (v->destination_angle > 360)? (v->destination_angle-360):(v->destination_angle);
    int j = _destAngletoId[int(da)];
    while (i != j)
    {
        v->position.push_back(make_pair(answerList[i]->getStartTime(), rad_to_degree(answerList[i]->getStartAngle())));
        i++;
        if (i == ra_valid_source_angle.size()) i = 0;
    }
    v->position.push_back(make_pair(answerList[j]->getStartTime(), rad_to_degree(answerList[j]->getStartAngle())));
    */
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
    
    for (int i = 0; i < _raSourceAngleList.size(); i++)
    {
        DLnode* nodeU;
        DLnode* nodeD;
        // find up and down //
        if (answerList[i] != NULL && _raSourceAngleList[i] != NULL)
        {
            if (_raSourceAngleList[i]->getStartTime() > answerList[i]->getStartTime())
            {
                nodeD = new DLnode(-1, INT_MIN, INT_MIN, degree_to_rad(ra_valid_source_angle[i]));
                nodeU = new DLnode(-1, _raSourceAngleList[i]->getStartTime(), _raSourceAngleList[i]->getEndTime(), _raSourceAngleList[i]->getStartAngle());
            }
            else
            {
                nodeD = _raSourceAngleList[i];
                while(nodeD->getNext() != _raSourceAngleList[i] && nodeD->getNext()->getStartTime() < answerList[i]->getStartTime())
                {
                    nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[i])
                    nodeU = new DLnode(-1, INT_MAX, INT_MAX, degree_to_rad(ra_valid_source_angle[i]));
                else
                    nodeU = new DLnode(-1, nodeU->getStartTime(), nodeU->getEndTime(), nodeU->getStartAngle());
                nodeD = new DLnode(-1, nodeD->getStartTime(), nodeD->getEndTime(), nodeD->getStartAngle());   
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


    for (int i = 0; i < _raSourceAngleList.size(); i++)
    {
        DLnode* node;
        // find overall skyline (each entry's prev) //
        if (_raSourceAngleList[i] == NULL)
        {
            node = new DLnode(-1, INT_MIN, INT_MIN, degree_to_rad(ra_valid_source_angle[i]));
        }
        else
        {
            //printSkyline(_raSourceAngleList[i]);
            //printf("%lf\n",  _raSourceAngleList[i]->getPrev()->getStartTime());
            node = _raSourceAngleList[i]->getPrev();
            node = new DLnode(-1, node->getStartTime(), node->getEndTime(), node->getStartAngle());
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
ra_mgr::canPlaceBetweenTwoSkyline(const vector<DLnode*> & answerList, const double time_interval, const int entryId)
{
    // printf("\ncanPlaceBetweenTwoSkyline\n");
    int sa_size = _raSourceAngleList.size();
    DLnode* upNode = _upSkyline, *downNode = _downSkyline;
    for (int i = 0; i < sa_size; i++)
    {
        if (answerList[i] != NULL)
        {   
            if ((upNode->getStartTime() - answerList[i]->getStartTime() < time_interval) && !(i == entryId && upNode->IsExit())) return false;
            if ((answerList[i]->getStartTime() - downNode->getStartTime() < time_interval) && !(i == entryId && downNode->IsExit())) return false;
        }
        upNode = upNode->getFront();
        downNode = downNode->getFront();
    }
    return true;
}

void 
ra_mgr::printSkyline(DLnode *node){
    DLnode *tmpNode = node;
    DLnode *startNode = node;
    while(tmpNode->getFront() != startNode){
        printf("%lf ", tmpNode->getStartTime());
        tmpNode = tmpNode->getFront();
    }
    printf("%lf\n", tmpNode->getStartTime());
}

DLnode* 
ra_mgr::clearSkyline(DLnode *skyline){
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


bool 
ra_mgr::LinesConflicted(DLnode* node1, DLnode* node2){
    if(node1->getStartTime() > node2->getStartTime() && node1->getEndTime() < node2->getEndTime()) return true;
    if(node1->getStartTime() < node2->getStartTime() && node1->getEndTime() > node2->getEndTime()) return true;
    return false;
}
