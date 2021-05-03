import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# line = { 'id': [ [pos1, pos2, ...], [t1, t2, ...] ], ... }, one color for one id key

def plot_line(all_v: dict, max_t: int):
    # title, label
    plt.title('Scheduling in Round-about')
    plt.xlabel('Position (unit: degree)')
    plt.ylabel('Time (unit: sec)')
    # plot every single line
    id_label = []
    for id in range(len(all_v)):
        # set point and its size
        plt.plot(all_v[str(id)][0], all_v[str(id)][1], marker='o', markersize=3)
        id_label.append(str(id))
    # plot vertical line in degree (90, 180, 270, 360)
    plt.vlines(90, 0, max_t, colors = "c", linestyles = "dashed")
    plt.vlines(180, 0, max_t, colors = "c", linestyles = "dashed")
    plt.vlines(270, 0, max_t, colors = "c", linestyles = "dashed")
    plt.vlines(360, 0, max_t, colors = "c", linestyles = "dashed")
    # set name for every line
    plt.legend(id_label)
    # show result, can comment it
    plt.show()

def main(filename): 
    # read output file (e.g. test.txt)
    all_v = dict()
    max_t = 0
    with open(filename, 'r') as f:
        print("Read output file......")
        for line in f:
            time, pos = [], []
            tmp = line.split(' ')
            tmp_id = tmp[0] #string
            # construct (x, y) for each vehicle, delete the '\n'
            for i in range(1, len(tmp)-1, 2):
                time.append(round(float(tmp[i]), 3))
                pos.append(round(float(tmp[i+1]), 3))
                # find max time
                if (round(float(tmp[i])) >= max_t):
                    max_t = round(float(tmp[i])+0.5)
            all_v[tmp_id] = [pos, time]
    plot_line(all_v, max_t)

if __name__ == '__main__':
    # e.g. "output/test.txt"
    filename = input('Please enter the filename to plot : ')
    main(filename)