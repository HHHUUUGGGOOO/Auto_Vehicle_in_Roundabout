import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from argparse import ArgumentParser

# line = { 'id': [ [pos1, pos2, ...], [t1, t2, ...] ], ... }, one color for one id key

def plot_line(all_v: dict, max_t: int, entry_list: list, exit_list: list):
    # title, label
    plt.title('Scheduling in Round-about')
    plt.xlabel('Position (unit: degree)')
    plt.ylabel('Time (unit: sec)')
    # plot every single line
    id_label = []
    # print(all_v)
    for id in all_v:
        # set point and its size
        plt.plot(all_v[id][0], all_v[id][1], marker='o', markersize=3)
        id_label.append(str(id))
    # plot vertical line in entry degree (90, 180, 270, 360)
    for entry in entry_list:
        plt.vlines(entry, 0, max_t, colors = "b", linestyles = "dashed")
    plt.vlines(360, 0, max_t, colors = "b", linestyles = "dashed")
    # plot vertical line in exit degree (90, 180, 270, 360)
    for exit in exit_list:
        plt.vlines(exit, 0, max_t, colors = "r", linestyles = "dashed")
    plt.vlines(360, 0, max_t, colors = "r", linestyles = "dashed")
    # set name for every line
    plt.legend(id_label)
    # show result, can comment it
    plt.show()

def main(filename_v, filename_ra): 
    # read output file (e.g. test.txt)
    all_v = dict()
    max_t = 0
    with open(filename_v, 'r') as f:
        print("Read output file......")
        for line in f:
            time, pos = [], []
            tmp = line.split(' ')
            if (len(tmp) == 1):
                continue
            tmp_id = tmp[0] #string
            # construct (x, y) for each vehicle, delete the '\n'
            for i in range(1, len(tmp)-1, 2):
                time.append(round(float(tmp[i]), 3))
                pos.append(round(float(tmp[i+1]), 3))
                if i+2 < len(tmp) -1 and float(tmp[i+1]) > float(tmp[i+3]):
                    time.append(round(float(tmp[i+2]), 3))
                    pos.append(360+round(float(tmp[i+3]), 3))
                # find max time
                if (round(float(tmp[i])) >= max_t):
                    max_t = round(float(tmp[i])+0.5)
            all_v[tmp_id] = [pos, time]
    # read round-about file (e.g. ra1.in)
    entry_list, exit_list = [], []
    count_line = 0
    with open(filename_ra, 'r') as f:
        print("Read round-about file......")
        for line in f:
            if (count_line == 2):
                tmp_entry = line.split(' ')
                for num in range(len(tmp_entry)-1):
                    entry_list.append(round(float(tmp_entry[num])))
            if (count_line == 4):
                tmp_exit = line.split(' ')
                for num in range(len(tmp_exit)-1):
                    exit_list.append(round(float(tmp_exit[num])))
            count_line += 1
    # plot
    plot_line(all_v, max_t, entry_list, exit_list)

def parse_args():
    parser = ArgumentParser()
    parser.add_argument(
            "--output_file",
            type=str,
            help="path to the output_file",
    )
    parser.add_argument(
            "--ra_file",
            type=str,
            help="path to the round-about file",
    )
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    # e.g. "output/test.txt"
    args = parse_args()
    main(args.output_file, args.ra_file)
    #filename_v = input('Please enter the output filename to plot : ')
    #filename_ra = input('Please enter the round-about filename to plot : ')
    #main(filename_v, filename_ra)