#!/usr/bin/env python

from email import header
from turtle import color
from matplotlib import pyplot as plt
import rospkg
import math
import csv
import numpy as np
import pandas as pd

class errorbar_drawer():
    
    def __init__(self):
        # self.means = []
        # self.max_errors = []
        # self.min_errors = []
        self.datalist = []
        self.rows = 0
        self.cols = 0
        self.rospack = rospkg.RosPack()
        self.csvfile = self.rospack.get_path("sim_localization")+"/no_algo.csv"
        self.fig,self.ax = plt.subplots()
        self.data2array()
        self.draw_noalgo()
    def data2array(self):
        with open(self.csvfile,'r') as f:
            reader = csv.reader(f)
            for row in reader:
                self.rows += 1
                self.cols = max(len(row),self.cols)
                self.datalist.append(row)
        self.dataarray = np.zeros((self.rows,self.cols))
        for i in range(self.rows):
            self.dataarray[i][0:len(self.datalist[i])] = np.array(self.datalist[i])
        self.means = self.dataarray.mean(axis = 0)
        self.min_errors = self.dataarray.min(axis = 0).reshape(1,-1)
        self.max_errors = self.dataarray.max(axis = 0).reshape(1,-1)
        self.errors  = np.concatenate((self.min_errors,self.max_errors),axis=0)
    
    def draw_noalgo(self):
        x = np.arange(0,self.cols,1)

        self.ax.set_xlabel('time/s')
        self.ax.set_ylabel('RMSE/m')
        self.ax.plot(x,self.means,linewidth=1,color='blue')
        # plt.plot(x,self.min_errors,linewidth=2,color='blue')
        # plt.plot(x,self.max_errors,linewidth=2,color='blue')
        self.ax.fill_between(x,self.min_errors.reshape(-1),self.max_errors.reshape(-1),facecolor='slateblue',alpha=0.3)

        # plt.errorbar(x,self.means,yerr=self.errors,fmt='-.',ecolor='r',color='b',elinewidth=1,capsize=2)
        plt.show()

if __name__ == '__main__':
    drawer = errorbar_drawer()
