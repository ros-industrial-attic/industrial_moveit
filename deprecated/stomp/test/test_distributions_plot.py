#!/usr/bin/env python

import matplotlib
matplotlib.use('GtkAgg')
import matplotlib.pyplot as plt
import numpy
import time
import os
import sys

class TestDistributionsPlotter:
    def __init__(self, directory, num_samples):
        self.directory = directory
        self.num_samples = int(num_samples)
        self.limit_timesteps = False

    def run(self):
        #fig = plt.figure(1, figsize=(2,2), dpi=128, frameon=True)
        fig = plt.figure(1, dpi=128, frameon=True)
        fig.set_figwidth(2)
        ax = plt.Axes(fig, [0., 0., 1., 1.])
        ax.set_axis_off()
        fig.add_axes(ax)
        #plt.axis([-1,1,0,1])
        for i in range(0, self.num_samples):
            file_name = self.directory+'/%d.txt'%(i)
            data = numpy.genfromtxt(file_name)
            if self.limit_timesteps:
                plt.plot(data[0:self.num_timesteps,0], data[0:self.num_timesteps,1], '-', linewidth=0.1, color='black')
            else:
                plt.plot(data[:,0], data[:,1], '-', linewidth=0.1, color='black')
       
        self.save_plot(iter,'')
        plt.close()

    def set_num_timesteps(self, num_timesteps):
        self.limit_timesteps = True
        self.num_timesteps=int(num_timesteps)

    
    def save_plot(self, iteration, suffix):
        file_name = self.directory+'/plot.svg'
        fig = matplotlib.pyplot.gcf()
        #fig.set_size_inches(4,4*1.04)
        #fig.set_dpi(128)
        a = fig.gca()
        #a.set_frame_on(True)
        a.set_xticks([])
        a.set_yticks([])
        a.set_aspect('equal')
        a.autoscale(tight=True)
        #plt.axis('tight')
        #plt.axis('equal')
        #plt.axis('on')
        #plt.axis([-0.5,0.5,-0.02,1.02])
        plt.savefig(file_name, bbox_inches='tight', pad_inches=0, dpi=128)#, pad_inches=0.01)


if __name__=='__main__':
    s = TestDistributionsPlotter(sys.argv[1], sys.argv[2])
    if len(sys.argv) >= 4:
        s.set_num_timesteps(sys.argv[3])
    s.run()
