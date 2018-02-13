#!/usr/bin/env python

# Based on RRT explained by Gabe Newell

import random, math
import matplotlib.pyplot as plt
from numpy import linspace

width = 100
length = 100
map_dim = [width, length]

q_init = [1,99]
q_goal = [100,0,20]
epsilon = 20

a = random.uniform(-1, 1)*2
gamma = random.uniform(-1, 1)*(math.pi)/2
dt = 0.1
v_max = 5
w_max = math.pi/2
theta_0 = math.pi/2
v_0 = 0
w_0 = 0

obstacles = []
with open('obstaclesH4.txt') as number:
        for k in number:
            obs_x, obs_y, rad = k.split(',')
            obs_x, obs_y, rad = int(obs_x), int(obs_y), float(rad)
            obstacles.append([obs_x, obs_y, rad])


def curves(n, new_node, theta_0, t):
    for i in range(0,100):
        displ = 0;
        x, y = q_init
        v = 0
        w = 0
        best = 1000
        theta = theta_0
        a = random.uniform(-1, 1)*2
        pos=x,y
        gamma = random.uniform(-1, 1)*(math.pi)/2
        prev_n = []
        next_n = []
        while displ < epsilon:

            x_new, y_new, theta_new, v_new, w_new = dynamics(x, y, theta, v, w, t, a,gamma)
            pos = x,y
            pos_new = x_new, y_new
            displ += euclidean(pos, pos_new)

            if (v_new > v_max):
                v_new = v_max
            elif (v_new < -v_max):
                v_new = -v_max
            if (w_new > w_max):
                w_new = w_max
            elif (w_new < -w_max):
                w_new = -w_max 

            x = x_new
            y = y_new
            theta = theta_new
            v = v_new
            w = w_new
        best_new = euclidean(pos_new, new_node)
        
        if best_new < best:
            best = best_new
            a_good = a
            g_good = gamma
    displ = 0
    a = a_good
    gamma = g_good
    x,y = n
    v = 0
    w = 0
    theta = theta_0
    while displ < epsilon:
        x_new, y_new, theta_new, v_new, w_new = dynamics(x, y, theta, v, w, t,a,gamma)
        pos = x,y
        pos_new = x_new, y_new
        displ += euclidean(pos, pos_new)
        if (v_new > v_max):
            v_new = v_max
        elif (v_new < -v_max):
            v_new = -v_max
        if (w_new > w_max):
            w_new = w_max
        elif (w_new < -w_max):
            w_new = -w_max

        pre = (x,y)
        post = (x_new, y_new)
        prev_n.append(pre)
        next_n.append(post)

        x = x_new
        y = y_new
        theta = theta_new
        n=x,y
        v = v_new
        w = w_new

    return prev_n, next_n, new_node

def dynamics(x_1, y_1, theta_1, v_1, w_1, t, a, gamma):
    v_i = v_1 + t*a
    w_i = w_1 + t*gamma
    theta_i = theta_1 + t*w_1
    x_i = x_1 + t*v_1*math.cos(theta_i)
    y_i = y_1 + t*v_1*math.sin(theta_i)
    return x_i, y_i, theta_i, v_i, w_i

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

def find_parent(n, source, nodes):
	i = nodes.index(n)
	parent = source[i]
	return parent

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
    source = []
    parent_path = []
    source.append(q_init)
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
        #prev_n, next_n, node_new = curves(n, node_new, theta_0, dt)
        #print prev_n, next_n
        #for l in range(0, len(prev_n)):
            #figure1, plt.plot([prev_n[l][0], next_n[l][0]], [prev_n[l][1], next_n[l][1]], color = 'yellow')

        nodes.append(node_new)
        source.append(n)
        print 'source'
        print source
        print 'nodes'
        print nodes

        figure1, plt.plot([n[0], node_new[0]], [n[1], node_new[1]], color = 'white')
        plt.pause(0.005)

    reverse = nodes[-1]
    parent_path.append(reverse)
    print parent_path
    while reverse != q_init:
    	parent = find_parent(reverse, source, nodes)
    	parent_path.append(parent)
    	figure1, plt.plot([reverse[0], parent[0]], [reverse[1], parent[1]], color = 'red')
    	reverse = parent




    while True:
        plt.pause(0.005)


if __name__ == '__main__':
    plot_nodes()