# roundabout input generator #

import sys
import math
import numpy as np
from pathlib import Path
from argparse import ArgumentParser, Namespace
import os

def main(args):
    print('Generate roundabout input file...')
    ra_max_capacity = int(2*math.pi*args.radius/(args.carLen*2)) #radius/car length
    
    #entry_space=360/NumOfEntry
    entry_list = [i*(360/args.NumOfEntry) for i in range(args.NumOfEntry)]

    #exit_sapce = 360/NumOfExit
    exit_list = [i*(360/args.NumOfExit) for i in range(args.NumOfExit)]

    if os.path.isfile(args.raFile):
        print(args.raFile, 'has already exist')
        check = input('Do you want to overwrite?(y/n): ')
        if check != 'y':
            print("Stop generating file ...")
            return

    with open(args.raFile, 'w+') as f:
        f.write("{:.1f} {:.1f} {:.1f} {:.1f} {:d}\n".format(args.radius, args.lowerVel ,args.upperVel, args.safetyMargin, ra_max_capacity))
        f.write('{:d}\n'.format(args.NumOfEntry))
        for i in range(args.NumOfEntry):
            f.write("{:.1f} ".format(entry_list[i]))
        f.write('\n')

        f.write('{:d}\n'.format(args.NumOfExit))
        for i in range(args.NumOfExit):
            f.write("{:.1f} ".format(exit_list[i]))
        f.write('\n')

    '''
    f=open(raFile, "w+")

    f.write("%.1f" %(entry_list[0]))
    for i in range(NumOfEntry-1):
            f.write(",%.1f" %(entry_list[i+1]))
    f.write('\n')
    
    f.write("%.1f" %(exit_list[0]))
    for i in range(NumOfExit-1):
            f.write(",%.1f" %(exit_list[i+1]))
    f.write('\n')
    f.close()
    '''
    print('Roundabout input file generated.')

def parse_args() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument('--NumOfEntry', type=int, default=4, help='Number of entries in roundabout.')
    parser.add_argument('--NumOfExit',  type=int, default=4, help='NUmber of exits in roundabout.')
    parser.add_argument('--raFile', type=str,  required = True, help='File to ra input file(user defined).')
    parser.add_argument('--radius', type=float, default=20.0, help='Radius of roundabout (unit:m)')
    parser.add_argument('--lowerVel', type=float, default=0.0, help='The minimum velocity allowed in roundabout (unit:m/s).')
    parser.add_argument('--upperVel', type=float, default=10.0, help='The maximum velocity allowed in roundabout (unit:m/s).')
    parser.add_argument('--safetyMargin', type=float, default=0.0, help='The minimum distance allowed in roundabout (unit:m).')
    parser.add_argument('--carLen', type=float, default=2.0, help='Car length use in compute maximum capacity (unit:m).')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()
    main(args)


