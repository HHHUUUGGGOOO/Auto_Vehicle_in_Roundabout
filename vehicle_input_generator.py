# input denerator #

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
    ra_file_name = os.path.join(args.ra_dir, args.ra_file_name)
    if not os.path.isfile(ra_file_name):
        print(ra_file_name, 'not exists')
        return
    with open(ra_file_name, 'r') as f:
        line = f.readline()
        line = line.split()  # There are four item in the first line, split them first
        ra_radius, ra_safety_velocity, ra_safety_margin, ra_max_capacity = float(line[0]), float(line[1]), float(line[2]), int(line[3])
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
        
        #print(ra_radius, ra_safety_velocity, ra_safety_margin, ra_max_capacity)
        #print(entry_list, exit_list)

    # vehicle 
    #num_of_v = int(input('Number of vehicles: '))
    #num_of_v_file = int(input('Number of vehicle input files: '))

    #expon_lamda=2

    print('Generate vehicles input file...')
    prefix = args.ra_file_name.split('.')[0]  # set prefix ra1.in -> ra1
    vehicle_file_name = os.path.join(args.vehicles_dir, prefix+'_' + args.vehicles_file_name)
    print(vehicle_file_name)
    
    if os.path.isfile(vehicle_file_name):
        print(vehicle_file_name, 'has already exist')
        check = input('Do you want to overwrite '+vehicle_file_name+'?(y/n) ')
        if check != 'y':
            print('Stop generating file ...')
            return

    v_id = 1
    v_earlist_arrival_time = 0
    v_source_angle = 0
    v_destination_angle = 0
    v_initial_velocity = 10; #unit=km/hr


    with open(vehicle_file_name, "w+") as f:
        for v_id in range(args.num_of_vehicles):
            v_source_angle = entry_list[ np.random.randint(0, num_of_entry)]
            v_destination_angle = exit_list[np.random.randint(0, num_of_entry)]
            while v_destination_angle == v_source_angle:
                v_destination_angle = exit_list[np.random.randint(0, num_of_entry)]
            y=np.random.ranf(size=None)
            k=stats.expon.ppf(y, loc=0 ,scale=1/args.expon_lamda)
            v_earlist_arrival_time+=k
            f.write("%i %.1f %.1f %.1f %.1f\n" %(v_id+1, v_earlist_arrival_time, v_source_angle, v_destination_angle, v_initial_velocity))    
    print('Vehicles input file generated.')
    return

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--ra_dir', type=Path, default='./input_ra/', help='Directory to the ra input files')
    parser.add_argument('--ra_file_name', type=str, default='ra1.in', help='Filename of the ra file')
    parser.add_argument('--vehicles_dir', type=Path, default='./input/', help='Directory to the vehicles input file')
    parser.add_argument('--vehicles_file_name', type=str, default='1.in', help='Filename of the vehicle file (e.g. 1.in)')
    parser.add_argument('--num_of_vehicles', type=int, default=5)
    print('here')
    parser.add_argument('--expon_lamda', type=float, default=2, help='Lamda of the exponantial distribution')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)
