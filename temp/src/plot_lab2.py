#!/usr/bin/env python


import os
import pickle
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    config_dir = os.path.join(os.path.dirname(__file__))
    config_dir = config_dir.replace('nodes', 'config')
    laser_data = pickle.load(open('add_data', 'rb'))
    comments = pickle.load(open('add_comment', 'rb'))
    laser_range= []
    for i in range(len(comments)):
        for x in range(360):
            laser_range.append(np.nan_to_num(laser_data[i].ranges[x]))
      	print laser_data[i].ranges
        plt.figure(i)
 #       plt.axis([0, 140, 0.0, 0.5])
        plt.plot(laser_data[i].ranges, 'b')
        plt.title(comments[i])
        plt.ylabel('Range')
    plt.show()

