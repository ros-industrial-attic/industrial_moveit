#!/usr/bin/env python

import matplotlib
matplotlib.use('GtkAgg')
import matplotlib.pyplot as plt
import numpy
import time
import os

class StompConstrainedTestPlotter:
    def __init__(self, directory):
        self.directory = directory

    def animate(self):
        self.num_time_steps = 100
        self.num_iters = 200
        self.num_joints = 100
    
        for iter in range(0, self.num_iters):
            fig = plt.figure(1, figsize=(2,2), dpi=128, frameon=True)
            ax = plt.Axes(fig, [0., 0., 1., 1.])
            ax.set_axis_off()
            fig.add_axes(ax)
            plt.axis([-1,1,0,1])
            self.animate_iter(iter)
            plt.close()

    def animate_iter(self, iter):
        file_name = self.directory+'/stomp_constrained_iter_%d.txt'%(iter)
        data = numpy.genfromtxt(file_name)
        start_color=numpy.array((0.0, 0.0, 1.0))
        end_color=numpy.array((0.0, 1.0, 0.0))
        for t in range(0, 12):
            frac = t/11.0
            color=(frac*end_color + (1-frac)*start_color)
            plt.plot(data[:,t*2+1], data[:,t*2]-0.01, color=color,
                    linestyle='-', marker='.', linewidth=0.5, markersize=1.0)
            plt.arrow(data[-1,t*2+1], data[-1,t*2]-0.01, 0.0, 0.00001, head_width=0.01, head_length=0.01, color=color, linewidth=0.2)
       
        start_point = plt.Circle((-0.4, 0.8), radius=0.02, fc='b', alpha=0.5)
        goal_point = plt.Circle((0.4, 0.8), radius=0.02, fc='g', alpha=0.5)
        obstacle = plt.Circle((0, 0.8), radius=0.2, fc='r', alpha=0.5)
        plt.gca().add_patch(start_point)
        plt.gca().add_patch(goal_point)
        plt.gca().add_patch(obstacle)
        self.save_plot(iter,'')
    
    def save_plot(self, iteration, suffix):
        file_name = self.directory+'/svg/%04d%s.svg'%(iteration,suffix)
        print file_name
        fig = matplotlib.pyplot.gcf()
        fig.set_size_inches(4,4*1.04)
        fig.set_dpi(128)
        a = fig.gca()
        a.set_frame_on(True)
        a.set_xticks([])
        a.set_yticks([])
        plt.axis('on')
        plt.axis([-0.5,0.5,-0.02,1.02])
        plt.savefig(file_name, bbox_inches='tight', pad_inches=0, dpi=128)#, pad_inches=0.01)


if __name__=='__main__':
    s = StompConstrainedTestPlotter('/tmp')
    s.animate()
