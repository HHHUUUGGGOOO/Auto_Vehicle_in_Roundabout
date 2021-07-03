/*****************************************************************************************
  FileName     [ case5_acceleraction.cpp ]
  PackageName  [ Auto Vehicle in Roundabout ]
  Synopsis     [ based on skyline problem to solve ]
  Author       [ Yen-Yu, Chen & Hugo, Chen & Yi-Jun, Huang]
  Professor    [ Iris Jiang & Chung-Wei, Lin ]
  Copyright    [ Copyleft(c) 2021, NTUEE, Taiwan ]
***************************************************************************************/

#include "ra_mgr.h"
#include <climits>

inline double velocity(DLnode* const n, double _raRadius)
{
    return _raRadius*(n->getAngleInterval()*(PI/180))/(n->getEndTime() - n->getStartTime());
}

inline double safety_time_interval(double _safety_margin, double vel)
{
    return _safety_margin/vel;
}

/*
    TODO:
(EG)   1. Insert checkUDskyline 在需要的地方 (54)
(胖呆)  2. 拔除 wait_ist, 只需 v_total \done
(Hugo)  3. 要考慮到 FCFS, 不能把後來的車排在先來的車前面 --> Vehicle 若 status = IN, priority 就是 eat, 所以進去某路口要判斷是否前面下面的車子 eat 都比他早,  可從 line 65 startTime 那邊開始找, 必須放在 eat 比他早的後面
            (可在每個路口存現在最晚被插進來的車的pointer (get時間), 再加上 (safety_margin / velocity)會是可插入的時間, velocity 直接 call INLINE)
(胖呆)  4. isAccelerate / isChange 要改 \done                                                 
(EG)    5. tmp_answerList 要刪掉改成 answer_list, canplacebetween 有些要改成 checkIfBetweenUDskyline 
(胖呆)  6. readfile 的 id 改成 string \done
(胖呆)  7. output file 或是中間存 position 的改成 answer_head (output 時要判斷是哪個 case 去看要用 position 還是 answer_head) \done but need furthur check (cannot compile now)
        8. 需要考慮到 capacity ??? capacity < (圓周/safety_margin)
(胖呆)  9. construct map & remove v_normal \done bot need furthur check      
*/

void 
ra_mgr::acceleration_solution_case_5()
{
    if (!v_total.size()) { cerr << "There is no vehicles to schedule !!" << endl; return; }

    int sa_size = ra_valid_source_angle.size();
    int v_size = v_total.size();
    sourceLatestVehicle.resize(sa_size);
    _raSourceAngleList.resize(sa_size);
    answerList.resize(sa_size); // temp answer list for one vehicle, answerList[0]: intersection_0 -> intersection_1, ... , answerList[sa_size-1] = intersection_{sa_size} -> intersection_0
    
    // note: v_total is sort by its time
    for (int current_v_id = 0; current_v_id < v_size; current_v_id++)
    {
        cerr << endl;
        cerr << "Vehicle id: " <<  v_total[current_v_id]->id << endl;
        double  startTime, endTime, startAngle, endAngle;
        // new added (Hugo)
        v_total[current_v_id]->velocityList.resize(sa_size);

        // Below _sourceAngletoId and _destAngletoId will conflict if they are not the same;
        double sa = v_total[current_v_id]->source_angle;
        int enterAngleId = _sourceAngletoId[int(sa)];
        double da = (v_total[current_v_id]->destination_angle > 360)? (v_total[current_v_id]->destination_angle-360):v_total[current_v_id]->destination_angle;
        int exitAngleId = _destAngletoId[int(da)];

        // compute answerList //
        for(int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
        {
            // new added (Hugo): reset velocityList
            v_total[current_v_id]->velocityList[currentAngleId] = v_total[current_v_id]->velocity;

            startAngle = degree_to_rad(ra_valid_source_angle[currentAngleId]);
            int nextIntersectionId = (currentAngleId + 1 == sa_size)? 0:currentAngleId+1; 
            endAngle = degree_to_rad(ra_valid_source_angle[nextIntersectionId]);
            if (endAngle < startAngle) { endAngle += 2*PI; }
            // new added (Hugo): startTime
            if (currentAngleId == enterAngleId) {
                // if NULL
                if (sourceLatestVehicle[currentAngleId] == NULL) { 
                    startTime = v_total[current_v_id]->earliest_arrival_time;
                    sourceLatestVehicle[currentAngleId] = v_total[current_v_id];
                }
                else {
                    startTime = sourceLatestVehicle[currentAngleId]->answer_head->getStartTime() + (ra_safety_margin / sourceLatestVehicle[currentAngleId]->velocityList[currentAngleId]);
                    sourceLatestVehicle[currentAngleId] = v_total[current_v_id];
                }
            }
            else { startTime = endTime; }
            // new added (Hugo): v_total[current_v_id]->velocity 改成 v_total[current_v_id]->velocityList[currentAngleId]   
            endTime = startTime + ra_radius*(endAngle-startAngle)/v_total[current_v_id]->velocityList[currentAngleId];
            // add "endAngle" to the new node
            answerList[currentAngleId] = new DLnode(v_total[current_v_id]->id, startTime, endTime, startAngle, endAngle, (currentAngleId == enterAngleId)/*start*/, (((currentAngleId+1)%sa_size) == exitAngleId)/*exit*/);
            // connect answerList
            if(currentAngleId != enterAngleId){
                int prevAngleId = (currentAngleId == 0)? sa_size - 1 : currentAngleId - 1;
                answerList[prevAngleId]->placePreviousTo(answerList[currentAngleId]);
            }
            cerr << "Start time: " << startTime << ", end time: " << endTime << ", start angle: " << startAngle << ", end angle" << endAngle << endl;
        }
        
        // compute upward and downward skyline //
        computeUDSkyline();

        // check if can fit between _downSkyline and _upSkyline else put based on _skyline
        // if can -> insert
        bool noAnswer = false;
        DLnode *nodeU, *nodeD;
        double timeUnit = 1e-3;
        while(!canPlaceBetweenTwoSkyline(enterAngleId, exitAngleId)) {
            
            nodeD = _raSourceAngleList[enterAngleId];
            if ((nodeD->getStartTime()-safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity)-timeUnit) > answerList[enterAngleId]->getStartTime()) {
                endTime = answerList[enterAngleId]->getStartTime() + timeUnit;
            }
            else
            {
                while(nodeD->getNext() != _raSourceAngleList[enterAngleId] && nodeD->getNext()->getStartTime() < answerList[enterAngleId]->getStartTime()) {
                    nodeD = nodeD->getNext();
                    while (nodeD->IsExit() && nodeD->getNext() != _raSourceAngleList[enterAngleId])
                        nodeD = nodeD->getNext();
                }
                nodeU = nodeD->getNext();
                if (nodeU == _raSourceAngleList[enterAngleId]) {
                    noAnswer = true;
                    break; // break while(!canPlaceBetweenTwoSkyline(enterAngleId, exitAngleId))
                }
                else {
                    if (nodeU->getStartTime() > answerList[enterAngleId]->getStartTime() + safety_time_interval(ra_safety_margin, v_total[current_v_id]->velocity) + timeUnit)
                        endTime = answerList[enterAngleId]->getStartTime() + timeUnit;
                    else endTime = nodeU->getStartTime() + safety_time_interval(ra_safety_margin, velocity(nodeU, ra_radius)); // depend on nodeU
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
                // new added (Hugo): v_total[current_v_id]->velocity 不用改成 v_total[current_v_id]->velocityList[currentAngleId] 
                endTime = startTime + ra_radius*(endAngle-startAngle)/v_total[current_v_id]->velocity;
                answerList[currentAngleId] = new DLnode(v_total[current_v_id]->id, startTime, endTime, startAngle, endAngle);
                // cerr << "startTime: " << startTime << " endTime: " << endTime << " angle: " << startAngle << endl;
            }
            // cerr << "New out: endTime: " << endTime << " angle: " << degree_to_rad(ra_valid_source_angle[j]) << endl;
            computeUDSkyline();
        }
    
        if (noAnswer)
        {
            // update answerList based on _skyline and insert
            printf("Vehicle %s can't place between skyline\n", v_total[current_v_id]->id.c_str());
            printSkyline(_skyline);
            DLnode * node = _skyline;
            for (int j = 0; j < enterAngleId; j++)
            {
                node = node->getFront();
            } // node is the correspondant node on _skyline wrt. enterAngleId
            // newly add (method to find answer) //
            // decide answerList at entry
            endTime = v_total[current_v_id]->earliest_arrival_time;
            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size, node = node->getFront())
            {
                startTime = max(endTime, node->getStartTime()+safety_time_interval(ra_safety_margin, velocity(node, ra_radius)));
                answerList[currentAngleId]->setStartTime(startTime);
                endTime = startTime + ra_radius*node->getAngleInterval()*(PI/180)/velocity(node, ra_radius);
                answerList[currentAngleId]->setEndTime(endTime);
            }

            // from entry update answer list //
            for (int currentAngleId = enterAngleId; currentAngleId != exitAngleId; currentAngleId = (currentAngleId+1)%sa_size)
            {
                int nextAngleId = (currentAngleId+1)%sa_size;
                answerList[currentAngleId]->setEndTime(answerList[nextAngleId]->getStartTime());
                if (velocity(answerList[currentAngleId], ra_radius) > ra_upper_velocity)
                {
                    // adjest endTime
                }
                else if (velocity(answerList[currentAngleId], ra_radius) < ra_lower_velocity)
                {
                    // adjust startTime
                }
            }
            
        }

        cout << "has answer" << endl;
        for (int currentAngleId = enterAngleId; true ; currentAngleId = (currentAngleId+1)%sa_size)
        {
            int nextAngleId = (currentAngleId+1)%sa_size;
            if (nextAngleId == exitAngleId) { break; }
            answerList[currentAngleId]->placeBehindOf(answerList[nextAngleId]);
        }
        
        cout << "update position" << endl;
        updatePosition(v_total[current_v_id]);
        cout << "insert to entry" << endl;
        insertToEntry(); // insert to _raSourceAngleList ans clear answerList

        // update _skyline //
        cout << "computeskyline" << endl;
        computeSkyline();

        //printSkyline(_skyline);
    }
    //for(int i = 0; i < _raSourceAngleList.size(); i++)
    //    printSkyline(_raSourceAngleList[i]);
}

void 
ra_mgr::insertToEntry()
{
    int sa_size = _raSourceAngleList.size();
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
    /// new: just link every answer and store to answer_head ///
    int ra_valid_destination_size = ra_valid_destination_angle.size();
    v->answer_head = answerList[_sourceAngletoId[int(v->source_angle)]];
    // printf("\nEnd update position\n");
}

void
ra_mgr::computeUDSkyline()
{
    // skyline must be new node different from _raSourceAngleList
    // skyline use cycle list for easy implement purpose

    // printf("\nCompute UD skyline\n");

    // clear skyline //
    _upSkyline = clearSkyline(_upSkyline);
    _downSkyline = clearSkyline(_downSkyline);

    // construct skyline //
    
    for (int i = 0; i < _raSourceAngleList.size(); i++)
    {
        DLnode* nodeU;
        DLnode* nodeD;
        // find up and down //
        // add "end_angle"
        // TOFIX
        double start_angle = ra_valid_source_angle[i];
        double end_angle = (i == _raSourceAngleList.size()-1) ? ra_valid_source_angle[0] : ra_valid_source_angle[i+1];
        if (answerList[i] != NULL && _raSourceAngleList[i] != NULL)
        {
            if (_raSourceAngleList[i]->getStartTime() > answerList[i]->getStartTime())
            {
                nodeD = new DLnode("", INT_MIN, INT_MIN, start_angle, end_angle);
                nodeU = new DLnode("", _raSourceAngleList[i]->getStartTime(), _raSourceAngleList[i]->getEndTime(), _raSourceAngleList[i]->getStartAngle(), _raSourceAngleList[i]->getEndAngle());
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
                    nodeU = new DLnode("", INT_MAX, INT_MAX, start_angle, end_angle);
                else
                    nodeU = new DLnode("", nodeU->getStartTime(), nodeU->getEndTime(), nodeU->getStartAngle(), nodeU->getEndAngle());
                nodeD = new DLnode("", nodeD->getStartTime(), nodeD->getEndTime(), nodeD->getStartAngle(), nodeD->getEndAngle());   
            }
        }
        else
        {
            nodeU = new DLnode("", INT_MAX, INT_MAX, start_angle, end_angle);
            nodeD = new DLnode("", INT_MIN, INT_MIN, start_angle, end_angle);
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
            // default skyline's "start_angle" and "end_angle"
            double start_angle = ra_valid_source_angle[i];
            double end_angle = (i == _raSourceAngleList.size()-1) ? ra_valid_source_angle[0] : ra_valid_source_angle[i+1];
            node = new DLnode("", INT_MIN, INT_MIN, start_angle, end_angle);
        }
        else
        {
            //printSkyline(_raSourceAngleList[i]);
            //printf("%lf\n",  _raSourceAngleList[i]->getPrev()->getStartTime());
            node = _raSourceAngleList[i]->getPrev();
            // add skyline's "end_angle"
            node = new DLnode("", node->getStartTime(), node->getEndTime(), node->getStartAngle(), node->getEndAngle());
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
ra_mgr::canPlaceBetweenTwoSkyline(const int entryId, const int exitId)
{
    // printf("\ncanPlaceBetweenTwoSkyline\n");
    int sa_size = _raSourceAngleList.size();
    DLnode* upNode = _upSkyline, *downNode = _downSkyline;
    // begin at "source angle"
    // cout << "1" << endl;
    for(int currentAngleId = 0; currentAngleId != entryId; currentAngleId++)
    {
        upNode = upNode->getFront();
        downNode = downNode->getFront();
    }
    // cout << "2" << endl;

    bool isAccelerate = false;
    bool isChange = false;
    double v_acc, v_decc, ENDTIME, _timeInterval;

    for (int i = entryId; i != exitId; i = (i+1)%sa_size)
    {
        // cout << "3" << endl;
        if (answerList[i] != NULL)
        {   
            // cout << "4" << endl;
            _timeInterval = ra_safety_margin/velocity(answerList[i], ra_radius);
            if ((upNode->getStartTime() - answerList[i]->getStartTime() < _timeInterval) && !(i == entryId && upNode->IsExit())) { // upskyline violation
                // cout << "5" << endl;
                if (i == entryId) { return false; }
                if (LinesConflicted(upNode, answerList[i])) { return false; }
                else {
                    // cout << "6" << endl;
                    int i_prevId = (i == 0) ? sa_size-1 : i-1;
                    v_acc = ra_radius * degree_to_rad(answerList[i_prevId]->getAngleInterval()) / (upNode->getStartTime() - time_interval - answerList[i_prevId]->getStartTime());
                    // if cannot accelerate
                    if ((v_acc > ra_upper_velocity) || (upNode->getStartTime() - _timeInterval < downNode->getStartTime() + time_interval))
                    { 
                        cout << "cannot accelerate" << endl;
                        return false; 
                    }
                    // if can accelerate, update answerList
                    else {
                        cout << "accelerate" << endl;
                        answerList[i_prevId]->setEndTime(upNode->getStartTime()-time_interval);
                        // update "start_time" and "end_time"
                        for (int j = i ; j != exitId; j = (j+1)%sa_size) {
                            int j_prevId = (j == 0) ? sa_size-1 : j-1;
                            answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                            double timeNeeded = ra_radius * degree_to_rad(answerList[j]->getAngleInterval())/ (_vId2VehicleMap[answerList[j]->getId()]->velocity);
                            answerList[j]->setEndTime( answerList[j]->getStartTime() + timeNeeded);
                        }
                        // new added (Hugo): change velocity and store in "velocityList"
                        _vId2VehicleMap[answerList[i]->getId()]->velocityList[i] = v_acc;
                        isAccelerate = true;
                        isChange = true;
                    }
                }
            }
            if ((answerList[i]->getStartTime() - downNode->getStartTime() < time_interval) && !(i == entryId && downNode->IsExit())) { // downskyline violation
                // deceleration
                // cout << "8" << endl;
                if (isAccelerate) { return false; }// we cannot acceleration and deceleration both 
                if (i == entryId) { return false; }
                if (LinesConflicted(downNode, answerList[i])) { return false; }
                else {
                    // cout << "9" << endl;
                    int i_prevId = (i == 0) ? sa_size-1 : i-1;
                    v_decc = ra_radius * degree_to_rad(answerList[i_prevId]->getAngleInterval()) / (downNode->getStartTime() + time_interval - answerList[i_prevId]->getStartTime());
                    // if cannot decelerate
                    if ((v_decc < ra_lower_velocity) || (downNode->getStartTime()+time_interval > upNode->getStartTime()-time_interval))
                    { 
                        cout << "cannot decelerate" << endl;
                        return false; 
                    }
                    // if can decelerate, update answerList
                    else {
                        cout << "decelerate" << endl;
                        answerList[i_prevId]->setEndTime(downNode->getStartTime() + time_interval);
                        // update "start_time" and "end_time"
                        for (int j = i ; j != exitId; j = (j+1)%sa_size) {
                            int j_prevId = (j == 0) ? sa_size-1 : j-1;
                            answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                            double timeNeeded = ra_radius * degree_to_rad(answerList[j]->getAngleInterval()) / (_vId2VehicleMap[answerList[j]->getId()]->velocity);
                            answerList[j]->setEndTime(answerList[j]->getStartTime() + timeNeeded);
                        }
                        // new added (Hugo): change velocity and store in "velocityList"
                        _vId2VehicleMap[answerList[i]->getId()]->velocityList[i] = v_decc;
                        isChange = true;
                    }
                }
            }
            isAccelerate = false;

            if ((answerList[i]->getEndTime() - downNode->getEndTime() < time_interval)) // accelerate
            {
                if (!downNode->IsExit())
                {
                    // wrong!! // downNode->getFront() is not the same vehicle
                    ENDTIME = ra_safety_margin/velocity(downNode->getFront(), ra_radius);
                }
                cerr << "here" << endl;
                v_acc = ra_radius * degree_to_rad(answerList[i]->getAngleInterval()) /(downNode->getEndTime() + ENDTIME - answerList[i]->getStartTime());

                // if cannot accelerate
                if ((v_acc > ra_upper_velocity) || (upNode->getEndTime()-time_interval < downNode->getEndTime()+time_interval))
                { 
                    cout << "cannot accelerate" << endl;
                    return false; 
                }
                // if can accelerate, update answerList
                else {
                    cout << "accelerate" << endl;
                    cerr << time_interval << endl;
                    answerList[i]->setEndTime(downNode->getEndTime() + ENDTIME);
                    // update "start_time" and "end_time"
                    for (int j = (i+1)%sa_size ; j != exitId; j = (j+1)%sa_size) {
                        int j_prevId = (j == 0) ? sa_size-1 : j-1;
                        answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                        double timeNeeded = ra_radius * degree_to_rad(answerList[j]->getAngleInterval()) / (_vId2VehicleMap[answerList[j]->getId()]->velocity);
                        answerList[j]->setEndTime(answerList[j]->getStartTime() + timeNeeded);
                        cerr << "ENDTIME: " << answerList[j]->getEndTime() << endl;
                    }
                    // new added (Hugo): change velocity and store in "velocityList"
                    _vId2VehicleMap[answerList[i]->getId()]->velocityList[i] = v_acc;
                    isAccelerate = true;
                    isChange = true;
                }
            }
            if ((upNode->getEndTime() - answerList[i]->getEndTime() < time_interval)) // deaccelerate
            {
                if (isAccelerate) return false;
                ENDTIME = max(ra_safety_margin/velocity(upNode, ra_radius), ra_safety_margin/velocity(answerList[i], ra_radius));
                answerList[i]->setEndTime(upNode->getEndTime() - ENDTIME);
                v_decc = velocity(answerList[i], ra_radius);
                // if cannot decelerate
                if ((v_decc < ra_lower_velocity) || (upNode->getEndTime() - time_interval < downNode->getEndTime() + time_interval))
                { 
                    cout << "cannot decelerate" << endl;
                    return false; 
                }
                // if can decelerate, update answerList
                else
                {
                    cout << "decelerate" << endl;
                    // update "start_time" and "end_time"
                    for (int j = j = (i+1)%sa_size ; j != exitId; j = (j+1)%sa_size) {
                        int j_prevId = (j == 0) ? sa_size-1 : j-1; 
                        answerList[j]->setStartTime(answerList[j_prevId]->getEndTime());
                        double timeNeeded = ra_radius * degree_to_rad(answerList[j]->getAngleInterval()) / (_vId2VehicleMap[answerList[j]->getId()]->velocity);
                        answerList[j]->setEndTime(answerList[j]->getStartTime() + timeNeeded);
                    }
                    // new added (Hugo): change velocity and store in "velocityList"
                    _vId2VehicleMap[answerList[i]->getId()]->velocityList[i] = v_decc;
                    isChange = true;
                }
            }

        }
        // cout << "11" << endl;
        upNode = upNode->getFront();
        downNode = downNode->getFront();
        isAccelerate = false;
    }
    if (!isChange) return true;
    return checkIfBetweenUDSkyline(time_interval, entryId, exitId);
}

bool 
ra_mgr::checkIfBetweenUDSkyline(const double time_interval, const int entryId, const int exitId){
    int sa_size = _raSourceAngleList.size();
    DLnode *upNode = _upSkyline;
    DLnode *downNode = _downSkyline;
    for (int i = 0; i < sa_size; i ++){ 
        if(exitId > entryId && i >= entryId && i < exitId && answerList[i] == NULL){ cerr << "Error: answerList[" << i << "] should not be NULL."; } 
        else if(exitId < entryId && (i >= entryId || i < exitId ) && answerList[i] == NULL){ cerr << "Error: answerList[" << i << "] should not be NULL."; }
        if (answerList[i] != NULL){
            if (answerList[i]->getStartTime() < downNode->getStartTime() + time_interval) { return false; }
            if (answerList[i]->getStartTime() > upNode->getStartTime() - time_interval) { return false; }
            if (answerList[i]->getEndTime() < downNode->getEndTime() + time_interval) { return false; }
            if (answerList[i]->getEndTime() > upNode->getEndTime() - time_interval) { return false; }
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
