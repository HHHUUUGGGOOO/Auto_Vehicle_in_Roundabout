/*****************************************************************************************
  FileName     [ case5_acceleraction.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ based on skyline problem to solve ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
***************************************************************************************/

#include "ra_mgr.h"
#include <typeinfo>

inline double velocity(DLnode* const n, double _raRadius)
{
    return _raRadius*(n->getAngleInterval()*(PI/180))/(n->getEndTime() - n->getStartTime());
}

inline double safety_time_interval(double _safety_margin, double vel)
{
    return _safety_margin/vel;
}

inline double computeNeededTime(double _raRadius, double radiun, double velocity){
    return _raRadius * radiun / velocity;
}

/*
    TODO:
        1. 需要考慮到 capacity ??? capacity < (圓周/safety_margin)   
*/

void 
ra_mgr::acceleration_solution_case_5()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }
    
    initGlobalVariables();
    int v_size = v_total.size();
    double  startTime, endTime, startAngle, endAngle;
    // note: v_total is sort by its time
    for (int current_v_id = 0; current_v_id < v_size; current_v_id++)
    {
        cerr << endl;
        cerr << "Vehicle id: " <<  v_total[current_v_id]->id << endl;

        // Below _sourceAngletoId and _destAngletoId will conflict if they are not the same;
        double sa = v_total[current_v_id]->source_angle;
        int enterAngleId = _sourceAngletoId[int(sa)];
        double da = (v_total[current_v_id]->destination_angle > 360)? (v_total[current_v_id]->destination_angle-360):v_total[current_v_id]->destination_angle;
        int exitAngleId = _destAngletoId[int(da)];

        // compute answerList //
        for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
        {
            startAngle = ra_valid_source_angle[currentAngleId];
            int nextIntersectionId = (currentAngleId + 1 == sa_size)? 0:currentAngleId+1; 
            endAngle = ra_valid_source_angle[nextIntersectionId];
            if (endAngle < startAngle) { endAngle += 360.0; } 
            // new added (Hugo): startTime
            if (currentAngleId == enterAngleId) {
                // if NULL
                if (sourceLatestVehicle[currentAngleId] == NULL) { 
                    startTime = v_total[current_v_id]->earliest_arrival_time;
                    sourceLatestVehicle[currentAngleId] = v_total[current_v_id];
                }
                else {
                    startTime = max(v_total[current_v_id]->earliest_arrival_time, sourceLatestVehicle[currentAngleId]->answer_head->getStartTime() + (ra_safety_margin / velocity(sourceLatestVehicle[currentAngleId]->answer_head, ra_radius)));
                    sourceLatestVehicle[currentAngleId] = v_total[current_v_id];
                }
            }
            else { startTime = endTime; }
            endTime = startTime + ra_radius * degree_to_rad(endAngle-startAngle)/v_total[current_v_id]->velocity;
            // add "endAngle" to the new node
            answerList[currentAngleId] = new DLnode(v_total[current_v_id]->id, startTime, endTime, ra_valid_source_angle[currentAngleId], ra_valid_source_angle[nextIntersectionId], (currentAngleId == enterAngleId)/*start*/, (((currentAngleId+1)%sa_size) == exitAngleId)/*exit*/);
            // connect answerList
            if(currentAngleId != enterAngleId){
                int prevAngleId = (currentAngleId == 0)? sa_size - 1 : currentAngleId - 1;
                answerList[currentAngleId]->placeInFrontOf(answerList[prevAngleId]);
            }
            cerr << "Start time: " << startTime << ", end time: " << endTime << ", start angle: " << startAngle << ", end angle: " << endAngle << endl;
        }
        
        // compute upward and downward skyline //
        computeUDSkyline();
        // check if can fit between _downSkyline and _upSkyline else put based on _skyline
        // if can -> insert
        bool noAnswer = false;
        DLnode *nodeU, *nodeD;
        while(!canPlaceBetweenTwoSkyline(enterAngleId, exitAngleId)) {
            nodeD = _raSourceAngleList[enterAngleId];
            if (nodeD == NULL)
            {
                endTime = answerList[enterAngleId]->getStartTime() + TIMEUNIT;
            }
            else if ((nodeD->getStartTime()-safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity)-TIMEUNIT) > answerList[enterAngleId]->getStartTime()) {
                endTime = answerList[enterAngleId]->getStartTime() + TIMEUNIT;
            }
            else
            {
                while(nodeD->getNext() != _raSourceAngleList[enterAngleId] && nodeD->getNext()->getStartTime() < answerList[enterAngleId]->getStartTime()) {
                    nodeD = nodeD->getNext();
                    // while (nodeD->IsExit() && nodeD->getNext() != _raSourceAngleList[enterAngleId])
                    //     nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                // cerr << "Endtime:"
                cerr<< "NodeU: " << nodeU->getStartTime() << " || NodeD: " << nodeD->getStartTime() << endl;
                if (nodeU == _raSourceAngleList[enterAngleId]) {
                    noAnswer = true;
                    break; // break while(!canPlaceBetweenTwoSkyline(enterAngleId, exitAngleId))
                }
                else {
                    cerr << "Hello" << endl;
                    cerr << "node U start time : " << nodeU->getStartTime() << ", answer Start time:" << answerList[enterAngleId]->getStartTime() << ", safety time: " << safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity) << endl;
                    if (nodeU->getStartTime() > answerList[enterAngleId]->getStartTime() + safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity) + TIMEUNIT){
                        endTime = answerList[enterAngleId]->getStartTime() + TIMEUNIT;
                    }
                    else endTime = nodeU->getStartTime() + safety_time_interval(ra_safety_margin, velocity(nodeU, ra_radius)); // depend on nodeU
                }
            }

            // compute new answerlist and check if it can put between skyline //
            for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                startTime = endTime;
                endTime = startTime + computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                answerList[currentAngleId]->setStartTime(startTime);
                answerList[currentAngleId]->setEndTime(endTime);
                // cerr << "startTime: " << startTime << " endTime: " << endTime << " angle: " << answerList[currentAngleId]->getStartAngle() << endl;
            }
            computeUDSkyline();
        }
    
        if (noAnswer)
        {
            // new method to find answer //
            // update answerList based on _skyline and insert
            printf("Vehicle %s can't place between skyline\n", v_total[current_v_id]->id.c_str());
            // printSkyline(_skyline);
            // decide startTime, endTime based on skyline //
            endTime = v_total[current_v_id]->earliest_arrival_time;
            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                if (_skyline[currentAngleId] == _lowerBoundSkyline[currentAngleId]) { 
                    startTime = endTime; 
                    endTime = startTime + computeNeededTime(ra_radius, degree_to_rad(_skyline[currentAngleId]->getAngleInterval()), ra_upper_velocity);
                }
                else { 
                    startTime = _skyline[currentAngleId]->getStartTime() + safety_time_interval(ra_safety_margin, velocity(_skyline[currentAngleId], ra_radius)); 
                    endTime = startTime + computeNeededTime(ra_radius, degree_to_rad(_skyline[currentAngleId]->getAngleInterval()), velocity(_skyline[currentAngleId], ra_radius));
                }
                answerList[currentAngleId]->setStartTime(startTime);                        
                answerList[currentAngleId]->setEndTime(endTime);
            }
            printSkyline(_skyline);
            printSkyline(answerList);



            // from entry update answer list //
            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                int nextAngleId = (currentAngleId+1) % sa_size;
                if (nextAngleId == exitAngleId) { 
                    // NOTE: case5 !!!! if violate velocity constraint?? //
                    if (velocity(answerList[currentAngleId], ra_radius) * (answerList[currentAngleId]->getEndTime() - _skyline[currentAngleId]->getEndTime()) < ra_safety_margin) {
                        answerList[currentAngleId]->setEndTime(answerList[currentAngleId]->getStartTime() + computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), velocity(_skyline[currentAngleId], ra_radius)));
                    }
                    break;
                }

                if (_skyline[nextAngleId] != _lowerBoundSkyline[nextAngleId] && _skyline[nextAngleId] != _upperBoundSkyline[nextAngleId] && velocity(answerList[currentAngleId], ra_radius)*(answerList[currentAngleId]->getEndTime()-_skyline[nextAngleId]->getStartTime()) < ra_safety_margin)
                {
                    cout << "here: " << currentAngleId << endl;
                    double vel = velocity(answerList[currentAngleId], ra_radius);
                    endTime = ra_safety_margin/vel;
                    cerr << "endTime: " << endTime << endl;
                    answerList[currentAngleId]->setEndTime(_skyline[nextAngleId]->getStartTime() + endTime);
                    startTime = answerList[currentAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), vel);
                    answerList[currentAngleId]->setStartTime(startTime);

                    cout << "print" << endl;
                    printSkyline(answerList);
                    // adjust prev: need to adjust all prev //
                    if (currentAngleId != enterAngleId){
                        double vel;
                        for (int prevAngleId = ((currentAngleId)?currentAngleId-1:sa_size-1); prevAngleId != enterAngleId; prevAngleId = ((prevAngleId)?prevAngleId-1:sa_size-1))
                        {
                            vel = velocity(answerList[prevAngleId], ra_radius);
                            answerList[prevAngleId]->setEndTime(startTime);
                            startTime = answerList[prevAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[prevAngleId]->getAngleInterval()), vel);
                            answerList[prevAngleId]->setStartTime(startTime);
                        }
                        vel = velocity(answerList[enterAngleId], ra_radius);
                        answerList[enterAngleId]->setEndTime(startTime);
                        startTime = answerList[enterAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[enterAngleId]->getAngleInterval()), vel);
                        answerList[enterAngleId]->setStartTime(startTime);     
                    }
                }

                if (answerList[currentAngleId]->getEndTime() > answerList[nextAngleId]->getStartTime())
                {
                    cout << "here1: " << currentAngleId << endl;
                    double vel = velocity(answerList[nextAngleId], ra_radius);
                    answerList[nextAngleId]->setStartTime(answerList[currentAngleId]->getEndTime());
                    endTime = answerList[nextAngleId]->getStartTime() + computeNeededTime(ra_radius, degree_to_rad(answerList[nextAngleId]->getAngleInterval()), vel);
                    answerList[nextAngleId]->setEndTime(endTime);
                }
                else
                {
                    cout << "here2: " << currentAngleId << endl;
                    answerList[currentAngleId]->setEndTime(answerList[nextAngleId]->getStartTime());
                    if (velocity(answerList[currentAngleId], ra_radius) > ra_upper_velocity + DELTA)
                    {
                        cout << "here3: " << currentAngleId << endl;
                        endTime = answerList[currentAngleId]->getStartTime() + computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), ra_upper_velocity);
                        answerList[currentAngleId]->setEndTime(endTime);
                        // adjest next: only need to adjust next one //
                        answerList[nextAngleId]->setStartTime(answerList[currentAngleId]->getEndTime());
                        endTime = answerList[nextAngleId]->getStartTime() + computeNeededTime(ra_radius, degree_to_rad(answerList[nextAngleId]->getAngleInterval()), ra_upper_velocity);
                        answerList[nextAngleId]->setEndTime(endTime);
                    }
                    else if (velocity(answerList[currentAngleId], ra_radius) < ra_lower_velocity - DELTA)
                    {
                        cout << "here4: " << currentAngleId << endl;
                        startTime = answerList[currentAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), ra_lower_velocity);
                        answerList[currentAngleId]->setStartTime(startTime);
                        // adjust prev: need to adjust all prev //
                        if (currentAngleId != enterAngleId){
                            double vel;
                            for (int prevAngleId = ((currentAngleId)?currentAngleId-1:sa_size-1); prevAngleId != enterAngleId; prevAngleId = ((prevAngleId)?prevAngleId-1:sa_size-1))
                            {
                                vel = velocity(answerList[prevAngleId], ra_radius);
                                answerList[prevAngleId]->setEndTime(startTime);
                                startTime = answerList[prevAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[prevAngleId]->getAngleInterval()), vel);
                                answerList[prevAngleId]->setStartTime(startTime);
                            }
                            vel = velocity(answerList[enterAngleId], ra_radius);
                            answerList[enterAngleId]->setEndTime(startTime);
                            startTime = answerList[enterAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[enterAngleId]->getAngleInterval()), vel);
                            answerList[enterAngleId]->setStartTime(startTime);     
                        }
                    }
                }
            }   
        }

        cout << "answerList" << endl;
        printSkyline(answerList);
        
        // cout << "update position" << endl;
        updatePosition(v_total[current_v_id]);
        // cout << "insert to entry" << endl;
        insertToEntry(); // insert to _raSourceAngleList ans clear answerList

        // update _skyline //
        // cout << "computeskyline" << endl;
        computeSkyline();
        cout << "after compute skyline" << endl;
        printSkyline(_skyline);
    }
}

void 
ra_mgr::insertToEntry()
{
    for (int currentAngleId = 0; currentAngleId < sa_size; currentAngleId++)
    {
        DLnode* node = _raSourceAngleList[currentAngleId];
        if (answerList[currentAngleId] != NULL)
        {
            if (node == NULL)
                _raSourceAngleList[currentAngleId] = answerList[currentAngleId];
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
                        node = node->getNext();
                    // node->getNext()->placeNextTo(answerList[currentAngleId]);
                    answerList[currentAngleId]->placeNextTo(node);
                }
            }
        }
        // clear answerList for next vehicle //
        answerList[currentAngleId] = NULL;
    }
}

void
ra_mgr::updatePosition(Vehicle* v)
{
    // printf("\nUpdate position\n");
    // just link every answer and store to answer_head //
    int ra_valid_destination_size = ra_valid_destination_angle.size();
    v->answer_head = answerList[_sourceAngletoId[int(v->source_angle)]];
    // printf("\nEnd update position\n");
}

void
ra_mgr::computeUDSkyline()
{
    // skyline must be new node different from _raSourceAngleList
    // skyline use cycle list for easy implement purpose

    // clear skyline //
    clearSkyline(_upSkyline);
    clearSkyline(_downSkyline);

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
                nodeD = _lowerBoundSkyline[i];
                nodeU = _raSourceAngleList[i];
            }
            else
            {
                nodeD = _raSourceAngleList[i];
                while(nodeD->getNext() != _raSourceAngleList[i] && nodeD->getNext()->getStartTime() < answerList[i]->getStartTime())
                {
                    // cerr << "UD while: " << nodeD->getId() << endl;
                    nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[i]) { nodeU = _upperBoundSkyline[i]; }  
            }
        }
        else
        {
            nodeU = _upperBoundSkyline[i];
            nodeD = _lowerBoundSkyline[i]; 
        }

        // insert to the last //
        // printf("\nInsert to the last\n");
        _upSkyline[i] = nodeU;
        _downSkyline[i] = nodeD;
    }
}

void
ra_mgr::computeSkyline()
{
    // printf("\nCompute skyline\n");
    // clear skyline //
    clearSkyline(_skyline);

    for (int i = 0; i < sa_size; i++) {
        DLnode* node;
        // find overall skyline (each entry's prev) //
        if (_raSourceAngleList[i] == NULL) { node = _lowerBoundSkyline[i]; }
        else { node = _raSourceAngleList[i]->getPrev(); }
        // insert to skyline[i] //
        _skyline[i] = node;
    }
}

bool
ra_mgr::canPlaceBetweenTwoSkyline(const int entryId, const int exitId)
{
    // printf("\ncanPlaceBetweenTwoSkyline\n");

    // NEW CANPLACE: define status (case 1-4 corresponding to our schema)
    short status = 0;
    short head_upNode_conflict = 1, head_downNode_conflict = 2, tail_upNode_conflict = 3, tail_downNode_conflict = 4;
    // begin at "source angle"
    bool isChange = false;
    cerr << "upSkyline" << endl;
    printSkyline(_upSkyline);
    cerr << "downSkyline" << endl;
    printSkyline(_downSkyline);
    cerr << "answerList" << endl;
    printSkyline(answerList);
    if (!(_upSkyline[entryId]->IsStart()) && velocity(_upSkyline[entryId]->getBehind(), ra_radius) * (_upSkyline[entryId]->getStartTime() - answerList[entryId]->getStartTime()) < ra_safety_margin) 
        return false; 
    for (int i = entryId; i != exitId; i = (i+1)%sa_size)
    {
        if (answerList[i] != NULL)
        {   
            int i_prevId = (i) ? i-1 : sa_size-1; 
            // case 1: head_upNode_conflict //
            if ( _upSkyline[i] != _upperBoundSkyline[i] && velocity(answerList[i], ra_radius)*(_upSkyline[i]->getStartTime()-answerList[i]->getStartTime()) < ra_safety_margin ) { 
                // cout << "status: 1" << endl;
                if (i == entryId) { return false; }
                if (status == tail_downNode_conflict) { return false; } 
                answerList[i_prevId]->setEndTime(_upSkyline[i]->getStartTime()-safety_time_interval(ra_safety_margin, _vId2VehicleMap[answerList[i]->getId()]->velocity));
                // if cannot accelerate
                if (velocity(answerList[i_prevId], ra_radius) > ra_upper_velocity+DELTA) { return false; }
                // if can accelerate, update answerList
                // cout << "accelerate" << endl;
                for (int j = i ; j != exitId; j = (j+1)%sa_size) {
                    int j_prevId = (j == 0) ? sa_size-1 : j-1;
                    answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                    double timeNeeded = computeNeededTime(ra_radius,  degree_to_rad(answerList[j]->getAngleInterval()), _vId2VehicleMap[answerList[j]->getId()]->velocity);
                    answerList[j]->setEndTime( answerList[j]->getStartTime() + timeNeeded);
                }
                // update and check UDskyline 
                computeUDSkyline();
                if (velocity(answerList[i_prevId], ra_radius)*(answerList[i_prevId]->getEndTime() - _downSkyline[i_prevId]->getEndTime()) < ra_safety_margin){
                    return false; 
                }
                if (_upSkyline[i_prevId] != _upperBoundSkyline[i_prevId] && velocity(_upSkyline[i_prevId], ra_radius)*(_upSkyline[i_prevId]->getEndTime() - answerList[i_prevId]->getEndTime()) < ra_safety_margin){
                    return false; 
                }
                
                status = head_upNode_conflict;
                isChange = true;   
            }
            // case 2-1: _downSkyline[i] is entry // (Too dirty here...= =)
            if ((i != entryId) && (_downSkyline[i]->IsStart()) && (velocity(answerList[i_prevId], ra_radius)*(answerList[i_prevId]->getEndTime() - _downSkyline[i]->getStartTime()) < ra_safety_margin)) {
                // cout << "status: 2-1" << endl;
                // cout << velocity(answerList[i_prevId], ra_radius)*(answerList[i_prevId]->getEndTime() - _downSkyline[i]->getStartTime()) << endl;
                // string str;
                // cin >> str;
                if (status == head_upNode_conflict) { return false; } // cannot acceleration and deceleration both 
                if (status == tail_upNode_conflict) { return false; }
                if (i == entryId) { return false; }
                answerList[i_prevId]->setEndTime(_downSkyline[i]->getStartTime() + ra_safety_margin*(_downSkyline[i]->getStartTime() - answerList[i_prevId]->getStartTime())/(ra_radius*degree_to_rad(answerList[i_prevId]->getAngleInterval())-ra_safety_margin));
                // if cannot decelerate
                if (velocity(answerList[i_prevId], ra_radius) < ra_lower_velocity-DELTA) { return false; }
                // if can decelerate, update answerList
                // cout << "decelerate" << endl;                
                for (int j = i ; j != exitId; j = (j+1)%sa_size) {
                    int j_prevId = (j == 0) ? sa_size-1 : j-1;
                    answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                    double timeNeeded = computeNeededTime(ra_radius,  degree_to_rad(answerList[j]->getAngleInterval()), _vId2VehicleMap[answerList[j]->getId()]->velocity);
                    answerList[j]->setEndTime(answerList[j]->getStartTime() + timeNeeded);
                }
                // update and check UDskyline 
                computeUDSkyline();
                if (velocity(answerList[i_prevId], ra_radius)*(answerList[i_prevId]->getEndTime() - _downSkyline[i_prevId]->getEndTime()) < ra_safety_margin) 
                    return false; 
                if (_upSkyline[i_prevId] != _upperBoundSkyline[i_prevId] && velocity(_upSkyline[i_prevId], ra_radius)*(_upSkyline[i_prevId]->getEndTime() - answerList[i_prevId]->getEndTime()) < ra_safety_margin) 
                    return false; 

                status = head_downNode_conflict;
                isChange = true;  
            }
            // case 2-2: head_downNode_conflict //
            if (_downSkyline[i] != _lowerBoundSkyline[i] && velocity(_downSkyline[i], ra_radius)*(answerList[i]->getStartTime() - _downSkyline[i]->getStartTime()) < ra_safety_margin) { 
                // cout << "status: 2-2" << endl;
                if (status == head_upNode_conflict) { return false; } // cannot acceleration and deceleration both 
                if (status == tail_upNode_conflict) { return false; }
                if (i == entryId) { return false; }
                answerList[i_prevId]->setEndTime(_downSkyline[i]->getStartTime() + safety_time_interval(ra_safety_margin, velocity(_downSkyline[i], ra_radius)));
                // if cannot decelerate
                if (velocity(answerList[i_prevId], ra_radius) < ra_lower_velocity-DELTA) { return false; }
                // if can decelerate, update answerList
                // cout << "decelerate" << endl;                
                for (int j = i ; j != exitId; j = (j+1)%sa_size) {
                    int j_prevId = (j == 0) ? sa_size-1 : j-1;
                    answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                    double timeNeeded = computeNeededTime(ra_radius,  degree_to_rad(answerList[j]->getAngleInterval()), _vId2VehicleMap[answerList[j]->getId()]->velocity);
                    answerList[j]->setEndTime(answerList[j]->getStartTime() + timeNeeded);
                }
                // update and check UDskyline 
                computeUDSkyline();
                if (velocity(answerList[i_prevId], ra_radius)*(answerList[i_prevId]->getEndTime() - _downSkyline[i_prevId]->getEndTime()) < ra_safety_margin) 
                    return false; 
                if (_upSkyline[i_prevId] != _upperBoundSkyline[i_prevId] && velocity(_upSkyline[i_prevId], ra_radius)*(_upSkyline[i_prevId]->getEndTime() - answerList[i_prevId]->getEndTime()) < ra_safety_margin) 
                    return false; 

                status = head_downNode_conflict;
                isChange = true;    
            }
            status = 0;
            // case 3: tail_upNode_conflict //
            if ( _upSkyline[i] != _upperBoundSkyline[i] && velocity(_upSkyline[i], ra_radius)*(_upSkyline[i]->getEndTime()-answerList[i]->getEndTime()) < ra_safety_margin) {
                answerList[i]->setEndTime(_upSkyline[i]->getEndTime()-safety_time_interval(ra_safety_margin, velocity(_upSkyline[i], ra_radius)));
                // if cannot accelerate
                // cout << "status: 3" << endl;
                if (velocity(answerList[i], ra_radius) > ra_upper_velocity+DELTA) { return false; }
                // if can accelerate, update answerList
                // cout << "accelerate" << endl;
                for (int j = (i+1)%sa_size ; j != exitId; j = (j+1)%sa_size) {
                    int j_prevId = (j) ? j-1 : sa_size-1;
                    answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                    double timeNeeded = computeNeededTime(ra_radius,  degree_to_rad(answerList[j]->getAngleInterval()), _vId2VehicleMap[answerList[j]->getId()]->velocity);
                    answerList[j]->setEndTime( answerList[j]->getStartTime() + timeNeeded);
                }
                computeUDSkyline();
                if (velocity(answerList[i], ra_radius)*(answerList[i]->getEndTime() - _downSkyline[i]->getEndTime()) < ra_safety_margin) {
                    return false;
                if (_upSkyline[i] != _upperBoundSkyline[i] && velocity(_upSkyline[i], ra_radius)*(_upSkyline[i]->getEndTime() - answerList[i]->getEndTime()) < ra_safety_margin)
                    return false;
                }
                status = tail_upNode_conflict;
                isChange = true;    
            }
            // case 4: tail_downNode_conflict // 
            if ( _downSkyline[i] != _lowerBoundSkyline[i] && velocity(answerList[i], ra_radius)*(answerList[i]->getEndTime()-_downSkyline[i]->getEndTime()) < ra_safety_margin) {
                // cout << "status: 4" << endl;
                answerList[i]->setEndTime((_downSkyline[i]->getEndTime() - answerList[i]->getStartTime())*ra_safety_margin/(ra_radius*degree_to_rad(answerList[i]->getAngleInterval())-ra_safety_margin)+ _downSkyline[i]->getEndTime());
                if (status == tail_upNode_conflict) { return false; }
                // if cannot decelerate
                if (velocity(answerList[i], ra_radius) < ra_lower_velocity-DELTA) { 
                    cout << velocity(answerList[i], ra_radius) << ", " << ra_lower_velocity << endl;
                    return false; }
                // if can decelerate, update answerList
                // cout << "decelerate" << endl;
                for (int j = (i+1)%sa_size ; j != exitId; j = (j+1)%sa_size) {
                    int j_prevId = (j) ? j-1 : sa_size-1;
                    answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                    double timeNeeded = computeNeededTime(ra_radius,  degree_to_rad(answerList[j]->getAngleInterval()), _vId2VehicleMap[answerList[j]->getId()]->velocity);
                    answerList[j]->setEndTime( answerList[j]->getStartTime() + timeNeeded);
                }
                computeUDSkyline(); 
                if (velocity(answerList[i], ra_radius)*(answerList[i]->getEndTime() - _downSkyline[i]->getEndTime()) < ra_safety_margin) { return false; }
                if (_upSkyline[i] != _upperBoundSkyline[i] && velocity(_upSkyline[i], ra_radius)*(_upSkyline[i]->getEndTime() - answerList[i]->getEndTime()) < ra_safety_margin) { return false; }
                status = tail_downNode_conflict;
                isChange = true;  
            }
        }
    }

    // printf("\nEndcanPlaceBetweenTwoSkyline\n");
    if (!isChange) return true;
    return checkIfBetweenUDSkyline(entryId, exitId);
}

bool 
ra_mgr::checkIfBetweenUDSkyline(const int entryId, const int exitId){
    // printSkyline(_downSkyline);
    // printSkyline(answerList);
    for (int i = 0; i < sa_size; i ++){ 
        if(exitId > entryId && i >= entryId && i < exitId && answerList[i] == NULL){ cerr << "Error: answerList[" << i << "] should not be NULL."; } 
        else if(exitId < entryId && (i >= entryId || i < exitId ) && answerList[i] == NULL){ cerr << "Error: answerList[" << i << "] should not be NULL."; }
        if (answerList[i] != NULL){
            // conflict constraint //
            if ( _downSkyline[i] != _lowerBoundSkyline[i] && LinesConflicted(_downSkyline[i],answerList[i])) return false;
            if ( _upSkyline[i] != _upperBoundSkyline[i] && LinesConflicted(_upSkyline[i],answerList[i])) return false;
            // safety margin constraint //
            if ( _downSkyline[i] != _lowerBoundSkyline[i] && answerList[i]->getStartTime() < _downSkyline[i]->getStartTime() + safety_time_interval(ra_safety_margin, velocity(_downSkyline[i], ra_radius))) { return false; }
            if ( _upSkyline[i] != _upperBoundSkyline[i] && answerList[i]->getStartTime() > _upSkyline[i]->getStartTime() - safety_time_interval(ra_safety_margin, velocity(answerList[i], ra_radius))) { return false; }
            if ( _downSkyline[i] != _lowerBoundSkyline[i] && answerList[i]->getEndTime() < _downSkyline[i]->getEndTime() + safety_time_interval(ra_safety_margin, velocity(answerList[i], ra_radius))) { return false; }
            if ( _upSkyline[i] != _upperBoundSkyline[i] && answerList[i]->getEndTime() > _upSkyline[i]->getEndTime() - safety_time_interval(ra_safety_margin, velocity(_upSkyline[i], ra_radius))) { return false; }
        }
    }
    return true;
}

void 
ra_mgr::printSkyline(vector<DLnode*> &skyline){
    for(int i = 0; i < sa_size; i++){
        if (skyline[i] == NULL) printf("(NULL) %c ", (i == sa_size-1)? '\n':' ');
        else printf("(%d, %lf -> %lf, %lf) %c ", i, skyline[i]->getStartTime(), skyline[i]->getEndTime(), velocity(skyline[i], ra_radius),  (i == sa_size-1)? '\n':' ');
    }
}

//TODO
void
ra_mgr::clearSkyline(vector<DLnode*> &skyline){
    for(int i = 0; i < sa_size; i++){
        skyline[i] = nullptr;
    }
}


bool 
ra_mgr::LinesConflicted(DLnode* node1, DLnode* node2){
    if(node1->getStartTime() > node2->getStartTime() && node1->getEndTime() < node2->getEndTime()) return true;
    if(node1->getStartTime() < node2->getStartTime() && node1->getEndTime() > node2->getEndTime()) return true;
    return false;
}

void 
ra_mgr::initGlobalVariables(){
    sa_size = ra_valid_source_angle.size();
    int v_size = v_total.size();
    sourceLatestVehicle.resize(sa_size);
    _raSourceAngleList.resize(sa_size);
    // NEW: resize UD skyline
    _upSkyline.resize(sa_size);
    _downSkyline.resize(sa_size);
    _skyline.resize(sa_size);
    // NEW: upperBound/lowerBound skyline
    _upperBoundSkyline.resize(sa_size);
    _lowerBoundSkyline.resize(sa_size);
    for(int i = 0; i < sa_size; i++){
        double start_angle = ra_valid_source_angle[i];
        double end_angle = ( i == sa_size-1 ) ? ra_valid_source_angle[0] : ra_valid_source_angle[i+1];
        _upperBoundSkyline[i] = new DLnode("", INT_MAX, INT_MAX, start_angle, end_angle);
        _lowerBoundSkyline[i] = new DLnode("", INT_MIN, INT_MIN, start_angle, end_angle);
    }
    answerList.resize(sa_size); // temp answer list for one vehicle, answerList[0]: intersection_0 -> intersection_1, ... , answerList[sa_size-1] = intersection_{sa_size} -> intersection_0

}