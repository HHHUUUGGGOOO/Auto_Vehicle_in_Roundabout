# vehicle input generator #

import sys
import math
import numpy as np
from scipy import stats
from numpy.random import exponential
from argparse import ArgumentParser, Namespace
from pathlib import Path
import os

def main(args):
    print('Read roundabout input file...')
    if not os.path.isfile(args.raFile):
        print(args.raFile, 'not exists')
        return
    with open(args.raFile, 'r') as f:
        line = f.readline()
        line = line.split()  # There are four item in the first line, split them first
        ra_radius, ra_lower_velocity, ra_upper_velocity, ra_safety_margin, ra_max_capacity = float(line[0]), float(line[1]), float(line[2]), float(line[3]), int(line[4])
        # read num of entry
        line = f.readline()
        num_of_entry = int(line)
        # read entry list
        line = f.readline()
        entry_list = [float(angle) for angle in line.split()]
        # ra
        line = f.readline()
        num_of_exit = int(line)
        line = f.readline()
        exit_list = [float(angle) for angle in line.split()]
        
        print(ra_radius, ra_lower_velocity, ra_upper_velocity, ra_safety_margin, ra_max_capacity)
        #print(entry_list, exit_list)

    print('Generate vehicles input file...')
    if os.path.isfile(args.vFile):
        print(args.vFile, 'has already exist')
        check = input('Do you want to overwrite '+args.vFile+'?(y/n) ')
        if check != 'y':
            print('Stop generating file ...')
            return

    v_id = v_source_angle =  v_destination_angle = v_earlist_arrival_time = v_velocity = 0

    with open(args.vFile, "w+") as f:
        print(args.NumOfVs)
        for v_id in range(args.NumOfVs):
            v_source_angle = entry_list[ np.random.randint(0, num_of_entry)]
            v_destination_angle = exit_list[np.random.randint(0, num_of_entry)]
            while v_destination_angle == v_source_angle:
                v_destination_angle = exit_list[np.random.randint(0, num_of_entry)]
            y=np.random.ranf(size=None)
            k=stats.expon.ppf(y, loc=0 ,scale=1/args.exponLamda)
            v_earlist_arrival_time+=k
            if args.ConstantVel == 'T':
                v_velocity = (ra_lower_velocity+ra_upper_velocity)/2
                f.write("%i %.1f %.1f %.1f %.1f\n" %(v_id+1, v_earlist_arrival_time, v_source_angle, v_destination_angle, v_velocity))  
            elif args.ConstantVel == 'F':
                v_velocity =  np.random.uniform(ra_lower_velocity, ra_upper_velocity)
                f.write("%i %.1f %.1f %.1f %.1f\n" %(v_id+1, v_earlist_arrival_time, v_source_angle, v_destination_angle, v_velocity)) 
    print('Vehicles input file generated.')
    return

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--raFile', type=str, required=True, help='File to the ra input files.')
    parser.add_argument('--vFile', type=str, required=True, help='File to the vehicle file (user defined).')
    parser.add_argument('--NumOfVs', type=int, default=10, help='Number of vehicles.')
    parser.add_argument('--ConstantVel', type=str, default='T', help='Vehicles are constant velocity or not.(T/F)')
    parser.add_argument('--exponLamda', type=float, default=2, help='Lamda of the exponantial distribution.')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)
