# output verifier #

import sys
import math
import numpy as np
from pathlib import Path
from argparse import ArgumentParser, Namespace
import os
import operator

class Vehicle:
    def __init__(self, id, eat, sa, da, iv):
        self._id = int(id)
        self._earlist_arrival_time = float(eat)
        self._source_angle = float(sa)
        self._destination_angle = float(da)
        self._initial_velocity = float(iv)
        self.time2angle={}
        self.timelist=[]
    
    def time2angle_dict(self, time, angle):
        # print("{:d} add angle {:.3f} at time {:.3f}".format(self._id, angle, time))
        self.time2angle[time]=angle
        self.timelist.append(time)
    def get_angle_by_time(self, time):
        if time in self.time2angle:
            return self.time2angle[time]
        else:
            return -1
    def get_timelist(self):
        return self.timelist
    def get_source_angle(self):
        return self._source_angle
    def get_destination_angle(self):
        return self._destination_angle

def main(args):
    ### read roundabout input ###
    with open(args.input_ra_dir, 'r') as f:
        print("Read roundaount input file...")
        line=f.readline()
        _ra_radius, _ra_safety_velocity, _ra_safety_margin, _ra_max_capacity=[float(i) for i in line.split()]
        _ra_max_capacity=int(_ra_max_capacity)

        _ra_number_of_source = int(f.readline())
        line=f.readline()
        _ra_source_angle = [float(i) for i in line.split()]
        if _ra_number_of_source != len(_ra_source_angle):
            print("Number of source lanes are given wrong!!", _ra_number_of_source)
            return

        _ra_number_of_destination = int(f.readline())
        line=f.readline()
        _ra_destination_angle=[float(i) for i in line.split()]
        if _ra_number_of_destination != len(_ra_destination_angle):
            print("Number of destination lanes are given wrong!!", _ra_number_of_destination)
            return
        
        # print roundabout information #
        print("--------------------------------")
        print("Roundabout informaion")
        print('File name: {}'.format(args.input_ra_dir))
        print("Radius: {:.2f}".format(_ra_radius))
        print("Safety velocity: {:.2f}".format(_ra_safety_velocity))
        print("Safety margin: {:.2f}".format(_ra_safety_margin))
        print("Maximum capacity: {:d}".format(_ra_max_capacity))
        print("--------------------------------\n")

    
    ### read vehicle input ###
    v_dict={} # key=id, value=vehicle
    with open(args.input_vehicle_dir, 'r') as f:
        print("Read vehicle input file...")
        for line in f:
            id, eat, sa, da, iv = [i for i in line.split()]
            v_dict[int(id)] = Vehicle(id, eat, sa, da, iv)
            print("Read Vehicle...(id = {:d})".format(int(id)))
        print("")

    '''
    t_dict: key=time, value=[[id,angle], [id,angle]] 
    1. length of value can check capacity
    2. if value sort by angle can check safety margin
    '''
    t_dict={} 
    time2id={}
    timelist=[]

    ### read output and verify some constraints ###
    with open(args.output, 'r') as f:
        print("Read output file...")
        for line in f:
            tmp=line.split(' ')
            tmp_id=int(tmp[0])
            # construct t_dict #
            for i in range(1, len(tmp),2):
                time=round(float(tmp[i]), 3)
                angle=round(float(tmp[i+1]), 3)
                v_dict[tmp_id].time2angle_dict(time, angle)
                if time in t_dict:
                    t_dict[time].append([tmp_id,angle])
                else:
                    t_dict[time]=[[tmp_id,angle]]
                    timelist.append(time)
    timelist.sort()
    for i in range(len(timelist)):
        time2id[timelist[i]]=i

    for vehicle in v_dict:
        # print("id= {:d}".format(vehicle))
        tmp_list=v_dict[vehicle].get_timelist()
        # print(tmp_list)

        # check source and destination angle
        if v_dict[vehicle].get_angle_by_time(tmp_list[0]) != v_dict[vehicle].get_source_angle():
            print("id: {:d}'s source angle is scheduled wrong!! ({} vs {})".format(vehicle, v_dict[vehicle].get_angle_by_time(tmp_list[0]), v_dict[vehicle].get_source_angle()))
            return
        
        if v_dict[vehicle].get_angle_by_time(tmp_list[len(tmp_list)-1]) != v_dict[vehicle].get_destination_angle():
            print("id: {:d}'s source angle is scheduled wrong!! ({} vs {})".format(vehicle, v_dict[vehicle].get_angle_by_time(tmp_list[len(tmp_list)-1]), v_dict[vehicle].get_destination_angle()))
            return
        
        # deal with middle 
        for i in range(0, len(tmp_list)-1):
            t1=time2id[tmp_list[i]]
            t2=time2id[tmp_list[i+1]]
            angle_unit=round((v_dict[vehicle].get_angle_by_time(tmp_list[i+1])-v_dict[vehicle].get_angle_by_time(tmp_list[i]))/(tmp_list[i+1]-tmp_list[i]),6)
            
            # TODO: compare _ra_safety_velocity and angle_unit to verify constraint #

            # print("{}, {}, {}, {}, {}".format(t2, t1, tmp_list[i+1], tmp_list[i], unit))
            for j in range(t1+1, t2):
                time_unit=timelist[j]-tmp_list[i]
                angle=v_dict[vehicle].get_angle_by_time(tmp_list[i])+angle_unit*time_unit
                t_dict[timelist[j]].append([vehicle,angle])
                print("id: {} insert angle {} at time {}".format(vehicle, angle, timelist[j]))
        
        for t in timelist:
            # sort t_dict's value list with value[i][1]
            t_dict[t]=sorted(t_dict[t], key=operator.itemgetter(1))

            # capacity constraint
            if len(t_dict[t]) > _ra_max_capacity:
                print("At time {} violate capacity constraint: {}".format(t, len(t_dict[t])))
                return
            
            # TODO: check _ra_safety_margin to verify constraint # 
    
    # output the last vehicle output time #
    print("--------------------------")
    print("last time: {}".format(timelist[len(timelist)-1]))
    print("Verification passed !!")
    print("--------------------------")

    return

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--input_ra_dir', type=Path, default='./input_ra/', help='Directory to the roundabout input')
    parser.add_argument('--input_vehicle_dir', type=Path, default='./input/', help='Directory to the vehicle input')
    parser.add_argument('--output', type=Path, default='output', help='Schedule output')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)