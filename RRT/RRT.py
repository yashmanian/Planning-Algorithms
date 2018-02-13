#!/usr/bin/env python

# Based on RRT explained by Steven Lavalle

import random, math
import matplotlib.pyplot as plt
from numpy import linspace

width = 100
length = 100
map_dim = [width, length]

q_init = [1,99]
q_goal = [100,0,20]
epsilon = 1

obstacles = []
with open('obstacles.txt') as number:
        for k in number:
            obs_x, obs_y, rad = k.split(',')
            obs_x, obs_y, rad = int(obs_x), int(obs_y), float(rad)
            obstacles.append([obs_x, obs_y, rad])

def euclidean(n1,n2):
    dist = math.sqrt(pow(n1[0]-n2[0],2)+pow(n1[1]-n2[1],2))
    return dist

def next_node(n1,n2):
    if euclidean(n1,n2) < epsilon:
        return n2
    else:
        angle = math.atan2(n2[1]-n1[1],n2[0]-n1[0])
        n_x = n1[0] + epsilon*math.cos(angle)
        n_y = n1[1] + epsilon*math.sin(angle)
        return n_x, n_y 

def node_collision_check(node_new, obstacles):
    for i in range(len(obstacles)):
        d2obs = euclidean(node_new, obstacles[i])
        if d2obs <= rad:
            cflag = 1
            return cflag
        else:
            cflag = 0 

def path_collision_check(n, node_new, obstacles):
    t = linspace(0, 1, 100)
    for j in range(len(t) - 1):
        px = (1 - t[j])*n[0] + t[j]*node_new[0]
        py = (1 - t[j])*n[1] + t[j]*node_new[1]
        cflag2 = node_collision_check((px, py), obstacles)
        if cflag2 == 1:
            return cflag2;

def plot_nodes():
    figure1, axes = plt.subplots(facecolor='black')
    axes.set_ylim((0, length))
    axes.set_xlim((0, width))
    axes.set_axis_bgcolor("black")
    q_init1 = plt.Circle((q_init[0], q_init[1]), 1, color='m', alpha=0.7)
    q_goal1 = plt.Circle((q_goal[0], q_goal[1]), q_goal[2], color='y', alpha=0.4)
    obs = figure1.add_subplot(111)
    obs.add_artist(q_init1)
    obs.add_artist(q_goal1)
    obstacle_plot = []
    for l in range(len(obstacles)):
        obstacle_plot = plt.Circle((obstacles[l][0], obstacles[l][1]), obstacles[l][2], color='b', alpha=0.5)
        obs.add_artist(obstacle_plot)
    figure1, plt.ion()

    nodes = []
    nodes.append(q_init)
    node_new = q_init
    while euclidean((q_goal[0], q_goal[1]), node_new) >= q_goal[2]:
        rand = random.random()*width, random.random()*length 
        n = nodes[0]
        for i in nodes:
            if euclidean(i, rand) < euclidean(n, rand):
                n = i
        node_new = next_node(n, rand)

        cflag = node_collision_check(node_new, obstacles)
        if cflag == 1:
            continue

        cflag2 = path_collision_check(n, node_new, obstacles)
        if cflag2 == 1:
            continue
        nodes.append(node_new)
        figure1, plt.plot([n[0], node_new[0]], [n[1], node_new[1]], color = 'white')
        plt.pause(0.0005)

    while True:
        plt.pause(0.005)


if __name__ == '__main__':
    plot_nodes()