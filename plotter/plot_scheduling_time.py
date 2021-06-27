import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser


def plot_time(x_id: list, t_case: list, title_name: str):
    # title, label
    plt.title('Exit Time for Each Vehicle (%s)' % (title_name.replace('.out', '')))
    plt.xlabel('vehicle id')
    plt.ylabel('Time (unit: sec)')
    case_label = ['case 1', 'case 2', 'case 3', 'case 4']
    # plot
    for i in range(len(case_label)):
        plt.plot(x_id, t_case[i], marker='o', markersize=3)
    # show
    plt.legend(case_label)
    plt.show()

def main(filename_v): 
    # read input file name, store (x, y) = (v_id, out_time)
    file_name = filename_v.replace('.in', '.out').replace('input/v_in/', '')
    output_case_1 = 'output/case1_' + file_name
    output_case_2 = 'output/case2_' + file_name
    output_case_3 = 'output/case3_' + file_name
    output_case_4 = 'output/case4_' + file_name
    # parameter
    x_id = []
    t_case1, t_case2, t_case3, t_case4 = [], [], [], []
    # case 1
    with open(output_case_1, 'r') as f_1:
        print("Read case 1 output file......")
        for line in f_1:
            tmp = line.split(' ')
            if (len(tmp) == 1):
                continue; 
            x_id.append(int(tmp[0]))
            t_case1.append(round(float(tmp[-3]), 3))
    # case 2
    with open(output_case_2, 'r') as f_2:
        print("Read case 2 output file......")
        for line in f_2:
            tmp = line.split(' ')
            if (len(tmp) == 1):
                continue; 
            t_case2.append(round(float(tmp[-3]), 3))        
    # case 3
    with open(output_case_3, 'r') as f_3:
        print("Read case 3 output file......")
        for line in f_3:
            tmp = line.split(' ')
            if (len(tmp) == 1):
                continue; 
            t_case3.append(round(float(tmp[-3]), 3)) 
    # case 4
    with open(output_case_4, 'r') as f_4:
        print("Read case 4 output file......")
        for line in f_4:
            tmp = line.split(' ')
            if (len(tmp) == 1):
                continue; 
            t_case4.append(round(float(tmp[-3]), 3)) 
    # plot
    t_case = [t_case1, t_case2, t_case3, t_case4]
    plot_time(x_id, t_case, file_name)

def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
            "--input_file",
            type=str,
            help="path to the input_file",
    )
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    # e.g. "input/v_in/ra4-4_30.in"
    args = parse_args()
    main(args.input_file)