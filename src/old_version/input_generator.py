# input denerator #

import sys
import math
import numpy as np
from scipy import stats
from argparse import ArgumentParser, Namespace

def main(args):

    # ra_info
    num_of_entry = int(input('Number of entries in roundabout:'))
    num_of_exit = int(input('Munber of exits in roundabout:'))

    print('Generate roundabout input file...')

    ra_radius = 20 #unit = m
    ra_safety_velocity = 25
    ra_safety_margin = 0
    ra_max_capacity = int(2*math.pi*ra_radius/5) #radius/car length

    entry_space=360/num_of_entry
    entry_list = [i*(360/num_of_entry) for i in range(num_of_entry)]

    exit_sapce = 360/num_of_exit
    exit_list = [i*(360/num_of_exit) for i in range(num_of_exit)]

    ra_index=2
    file_name = './input_ra/ra' + str(ra_index) + ".in"

    f=open(file_name, "w+")

    f.write("%.1f %.1f %.1f %i\n" %(ra_radius, ra_safety_velocity, ra_safety_margin, ra_max_capacity))
    f.write("%.1f" %(entry_list[0]))
    for i in range(num_of_entry-1):
            f.write(",%.1f" %(entry_list[i+1]))
    f.write('\n')

    f.write("%.1f" %(exit_list[0]))
    for i in range(num_of_exit-1):
            f.write(",%.1f" %(exit_list[i+1]))
    f.write('\n')

    f.close()
    print('Roundabout input file generated.')

    # vehicle 
    num_of_v = int(input('Number of vehicles: '))
    num_of_v_file = int(input('Number of vehicle input files: '))

    expon_lamda=2

    print('Generate vehicles input file...')

    for v_index in range(0, num_of_v_file):
            v_id = 1;
            v_earlist_arrival_time = 0;
            v_source_angle = 0;
            v_destination_angle = 0;
            v_initial_velocity = 10; #unit=km/hr

            file_name = './input/' + str(v_index+1) + ".in"
            f=open(file_name, "w+")
            for i in range(num_of_v):
                    v_source_angle = entry_list[ np.random.randint(0,num_of_entry) ]
                    v_destination_angle = exit_list[ np.random.randint(0,num_of_exit) ]
                    y=np.random.ranf(size=None)
                    k=stats.expon.ppf(y, loc=0 ,scale=1/expon_lamda)
                    v_earlist_arrival_time+=k
                    f.write("%i %.1f %.1f %.1f %.1f\n" %(v_id, v_earlist_arrival_time, v_source_angle, v_destination_angle, v_initial_velocity))
                    v_id+=1;
            f.close()

    print('Vehicles input file generated.')

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--num_of_entry')
    parser.add_argument('--num_of_exit')
    parser.add_argument('--ra_radius', type=float, default=20.0, help='Radius of roundabout (unit:m)')
    parser.add_argument('--ra_input_dir', type=Path, default='./input_ra/', help='The directory to the roundabout input')
    parser.add_argument('--vehicles_input_dir', type=Path, default='./input/', help='The directory of the vehicles input')
    parser.add_argument('--ra_input_dir')
    parser.add_argument('--ra_input_dir')

    parser.add_argument('num_of_entry')


if __name__ == '__main__':
    args = parse_args()
    main(args)
