# input denerator #

import sys
import math
import numpy as np
from pathlib import Path
#from scipy import stats
from argparse import ArgumentParser, Namespace
import os

def main(args):


    '''
    # ra_info
    num_of_entry = int(input('Number of entries in roundabout:'))
    num_of_exit = int(input('Munber of exits in roundabout:'))
    '''
    print('Generate roundabout input file...')
    '''
    ra_radius = 20 #unit = m
    ra_safety_velocity = 25
    ra_safety_margin = 0
    '''
    ra_max_capacity = int(2*math.pi*args.ra_radius/5) #radius/car length
    
    #entry_space=360/num_of_entry
    entry_list = [i*(360/args.num_of_entry) for i in range(args.num_of_entry)]

    #exit_sapce = 360/num_of_exit
    exit_list = [i*(360/args.num_of_exit) for i in range(args.num_of_exit)]

    #ra_index=2
    file_name = os.path.join(args.input_ra_dir, args.ra_file_name)
    #file_name = './input_ra/ra' + str(ra_index) + ".in"
    if os.path.isfile(file_name):
        print(file_name, 'has already exist')
        check = input('Do you want to overwrite?(y/n): ')
        if check != 'y':
            print("Stop generating file ...")
            return
    with open(file_name, 'w+') as f:
        f.write("{:.1f} {:.1f} {:.1f} {:d}\n".format(args.ra_radius, args.ra_safety_velocity, args.ra_safety_margin, ra_max_capacity))
        f.write('{:d}\n'.format(args.num_of_entry))
        for i in range(args.num_of_entry):
            f.write("{:.1f} ".format(entry_list[i]))
        f.write('\n')

        f.write('{:d}\n'.format(args.num_of_exit))
        for i in range(args.num_of_exit):
            f.write("{:.1f} ".format(exit_list[i]))
        f.write('\n')

    '''
    f=open(file_name, "w+")

    f.write("%.1f" %(entry_list[0]))
    for i in range(num_of_entry-1):
            f.write(",%.1f" %(entry_list[i+1]))
    f.write('\n')
    
    f.write("%.1f" %(exit_list[0]))
    for i in range(num_of_exit-1):
            f.write(",%.1f" %(exit_list[i+1]))
    f.write('\n')
    f.close()
    '''
    print('Roundabout input file generated.')

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--input_ra_dir', type=Path, default='./input_ra/', help='Directory to the roundabout input')
    parser.add_argument('--num_of_entry', type=int, default=4, help='Number of entries in roundabout')
    parser.add_argument('--num_of_exit',  type=int, default=4, help='NUmber of exits in roundabout')
    parser.add_argument('--ra_file_name', type=str,  default='ra1.in', help='file name of the ra input file')
    parser.add_argument('--ra_radius', type=float, default=20.0, help='Radius of roundabout (unit:m)')
    parser.add_argument('--ra_safety_velocity', type=float, default=25.0)
    parser.add_argument('--ra_safety_margin', type=float, default=0.0)

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)


