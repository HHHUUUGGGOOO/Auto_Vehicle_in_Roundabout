/*****************************************************************************************
  FileName     [ case4.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ based on skyline problem to solve ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
***************************************************************************************/

#include "ra_mgr.h"

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

void 
ra_mgr::constant_velocity_skyline_solution_case_4()
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
        while(!checkIfBetweenUDSkyline(enterAngleId, exitAngleId)) {
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
                    // FIXME: Newly removed 
                    // while (nodeD->IsExit() && nodeD->getNext() != _raSourceAngleList[enterAngleId])
                    //    nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[enterAngleId]) {
                    noAnswer = true;
                    break; // break while(!canPlaceBetweenTwoSkyline(enterAngleId, exitAngleId))
                }
                else {
                    if (nodeU->getStartTime() > answerList[enterAngleId]->getStartTime() + safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity) + TIMEUNIT)
                        endTime = answerList[enterAngleId]->getStartTime() + TIMEUNIT;
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
                    endTime = startTime + computeNeededTime(ra_radius, degree_to_rad(_skyline[currentAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                }
                else { 
                    // difference: base on skyline but use vehicle's initial velocity //
                    if (velocity(_skyline[currentAngleId], ra_radius) < v_total[current_v_id]->velocity)
                    {
                        endTime = _skyline[currentAngleId]->getEndTime() + safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity);
                        startTime = endTime - computeNeededTime(ra_radius, degree_to_rad(_skyline[currentAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                    }
                    else
                    {
                        startTime = _skyline[currentAngleId]->getStartTime() + safety_time_interval(ra_safety_margin, velocity(_skyline[currentAngleId], ra_radius)); 
                        endTime = startTime + computeNeededTime(ra_radius, degree_to_rad(_skyline[currentAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                    }
                }
                answerList[currentAngleId]->setStartTime(startTime);                        
                answerList[currentAngleId]->setEndTime(endTime);
            }

            // from entry update answer list //
            // difference : velocity constraint (constant velocity) //
            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                int nextAngleId = (currentAngleId+1) % sa_size;
                if (nextAngleId == exitAngleId) break;
                if (answerList[currentAngleId]->getEndTime() > answerList[nextAngleId]->getStartTime())
                {
                    answerList[nextAngleId]->setStartTime(answerList[currentAngleId]->getEndTime());
                    endTime = answerList[nextAngleId]->getStartTime() + computeNeededTime(ra_radius, degree_to_rad(answerList[nextAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                    answerList[nextAngleId]->setEndTime(endTime);
                }
                else if (answerList[currentAngleId]->getEndTime() < answerList[nextAngleId]->getStartTime())
                {
                    answerList[currentAngleId]->setEndTime(answerList[nextAngleId]->getStartTime());
                    startTime = answerList[currentAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[currentAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                    answerList[currentAngleId]->setStartTime(startTime);
                    // adjust prev: need to adjust all prev //
                    if (currentAngleId != enterAngleId){
                        for (int prevAngleId = ((currentAngleId)?currentAngleId-1:sa_size-1); prevAngleId != enterAngleId; prevAngleId = ((prevAngleId)?prevAngleId-1:sa_size-1))
                        {
                            answerList[prevAngleId]->setEndTime(startTime);
                            startTime = answerList[prevAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[prevAngleId]->getAngleInterval()), v_total[current_v_id]->velocity);
                            answerList[prevAngleId]->setStartTime(startTime);
                        }   
                        answerList[enterAngleId]->setEndTime(startTime);
                        answerList[enterAngleId]->setStartTime(answerList[enterAngleId]->getEndTime() - computeNeededTime(ra_radius, degree_to_rad(answerList[enterAngleId]->getAngleInterval()), v_total[current_v_id]->velocity));
                    }
                }
            }   
        }
        
        cout << "update position" << endl;
        updatePosition(v_total[current_v_id]);
        cout << "insert to entry" << endl;
        insertToEntry(); // insert to _raSourceAngleList ans clear answerList

        // update _skyline //
        cout << "computeskyline" << endl;
        computeSkyline();
        //printSkyline(_skyline, sa_size);
    }
    //for(int i = 0; i < _raSourceAngleList.size(); i++)
    //    printSkyline(_raSourceAngleList[i], sa_size);
}