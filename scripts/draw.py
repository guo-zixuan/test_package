#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def loadData(filepath):
    fr = open(filepath,'r+')
    lines = fr.readlines()
    length = len(lines)
    time = []
    x = []
    y = []
    z = []
    for line in lines:
        items = line.strip().split(',')
        time.append(float(items[0]))
        x.append(float(items[1]))
        y.append(float(items[2]))
        z.append(float(items[3]))
    
    return time, x, y, z, length


if __name__ == '__main__':
    time_1,x_1,y_1,z_1,length_1 = loadData('/home/gzx/slam_ws/src/my_test_package/data/new/gazeboOdom.txt')
    time_2,x_2,y_2,z_2,length_2 = loadData('/home/gzx/slam_ws/src/my_test_package/data/old/lowLidarOdom.txt')
    time_3,x_3,y_3,z_3,length_3 = loadData('/home/gzx/slam_ws/src/my_test_package/data/new/lowLidarOdom.txt')

    plt.figure()
    plt.xlabel('time/s')
    plt.ylabel('x/m')
    plt.title('Comparison chart of x-axis error')

    plt.plot(time_1,x_1,color="green",linestyle='--')
    plt.plot(time_2,x_2,color="blue",linestyle='-')
    plt.plot(time_3,x_3,color="red",linestyle=':')

    plt.legend(('ground truth x', 'lego loam x','lego loam+imu x'), loc='upper right')

    plt.figure()
    plt.xlabel('time/s')
    plt.ylabel('y/m')
    plt.title('Comparison chart of y-axis error')

    plt.plot(time_1,y_1,color="green",linestyle='--')
    plt.plot(time_2,y_2,color="blue",linestyle='-')
    plt.plot(time_3,y_3,color="red",linestyle=':')

    plt.legend(('ground truth y', 'lego loam y','lego loam+imu y'), loc='upper right')

    plt.figure()
    plt.xlabel('time/s')
    plt.ylabel('z/m')
    plt.title('Comparison chart of z-axis error')

    plt.plot(time_1,z_1,color="green",linestyle='--')
    plt.plot(time_2,z_2,color="blue",linestyle='-')
    plt.plot(time_3,z_3,color="red",linestyle=':')

    plt.legend(('ground truth z', 'lego loam z','lego loam+imu z'), loc='upper right')


    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot3D(x_1, y_1, z_1, 'green')
    ax.plot3D(x_2, y_2, z_2, 'blue')
    ax.plot3D(x_3, y_3, z_3, 'red')


    plt.show()

