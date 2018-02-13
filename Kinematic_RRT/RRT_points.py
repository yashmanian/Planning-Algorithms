import random, math
import matplotlib.pyplot as plt  # Plotting
import numpy as np
import csv
writer = csv.writer(open("path5.csv", 'w'))

# Map parameters
width = 100
length = 100
mapsize = [width, length]

# Map states
q_init = [40, 40]
theta_init = math.pi/4
q_goal = [0, 0, 20]
epsilon = 20
obstacles = []

# Dynamic constraints
v_max = 5
w_max = math.pi/2
v_0 = 0
w_0 = 0

# Read obstacles
with open('obstaclesH5.txt') as number:
        for k in number:
            obstacle_x, obstacle_y, radius = k.split(',')
            obstacle_x, obstacle_y, radius = int(obstacle_x), int(obstacle_y), float(radius)
            obstacles.append([obstacle_x, obstacle_y, radius])

robot = []
with open('H5_robot.txt') as vertex:
    for j in vertex:
        v_x, v_y = j.split(',')
        robot.append([v_x, v_y])

# Locate Source node index
def find_parent(n, source, nodes):
	i = nodes.index(n)
	parent = source[i]
	return parent


def homogenous(t_x, t_y, theta, x, y):
    coordinate = np.matrix('{}; {}; {}'.format(x, y, 1))
    n1 = math.cos(theta)
    n2 = math.sin(theta)
    transform = np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(n1, -n2, t_x, n2, n1, t_y, 0, 0, 1))
    p_1 = transform*coordinate
    return p_1[0,0], p_1[1,0]


# Euclidean distance between theo nodes
def euclidean(n1,n2):
	dist2node = math.sqrt(pow(n1[0]-n2[0],2)+pow(n1[1]-n2[1],2))
	return dist2node

# Check if the next node is within range epsilon
def next_node(n1,n2):
    if euclidean(n1,n2) < epsilon:
        return n2
    else:
        theta = math.atan2(n2[1]-n1[1],n2[0]-n1[0])
        n1_x = n1[0] + epsilon*math.cos(theta)
        n1_y = n1[1] + epsilon*math.sin(theta)
     	return n1_x, n1_y

# Model for 1/2 car model
def car_model(x, y, theta, v, w, t, a, gamma):
	v_next = v + t*a
	w_next = w + t*gamma
	theta_next = theta + t*w
	x_next = x + t*v*math.cos(theta)
	y_next = y + t*v*math.sin(theta) 
	return x_next, y_next, theta_next, v_next, w_next


#Check if node collides with obstacles
def node_collision_check(n, obstacles):
    for i in range(len(obstacles)):
        dist2obs = euclidean(n, obstacles[i])
        if dist2obs <= radius+0.5:
            cflag = 1
            return cflag
        if n[0] <= 0 or n[0] >= width or n[1] <= 0 or n[1] >= length:
            cflag = 1
            return cflag
        else:
            cflag = 0


def point_collision_check(n,obstacles):
    for i in range(len(obstacles)):
    	for j in range(len(n)):
    		dist2obs = euclidean(n[j], obstacles[i])
    		if dist2obs <= radius+0.5:
        		cflag = 1
        	return cflag
    		if n[0] <= 0 or n[0] >= width or n[1] <= 0 or n[1] >= length:
        		cflag = 1
        		return cflag
    		else:
        		cflag = 0


# Check if path to node has collisions
def path_collision_check(n, node_new, obstacles):
    t = linspace(0, 1, 100)
    for j in range(len(t) - 1):
        px = (1 - t[j])*n[0] + t[j]*node_new[0]
        py = (1 - t[j])*n[1] + t[j]*node_new[1]
        cflag2 = node_collision_check((px, py), obstacles)
        if cflag2 == 1:
            return cflag2;

# Plot trajectories
def plot_nodes():
# Setup figure for plot
	figure, ax = plt.subplots(facecolor='black')
	ax.set_ylim((0, length))
	ax.set_xlim((0, width))
	ax.set_axis_bgcolor("black")
	q_init1 = plt.Circle((q_init[0], q_init[1]), 1, color='b', alpha=0.7)
	q_goal1 = plt.Circle((q_goal[0], q_goal[1]), q_goal[2], color='g', alpha=0.4)
	ax.add_artist(q_init1)
	ax.add_artist(q_goal1)
	obstacle_area = []
	size_obst = len(obstacles)
	for k in range(size_obst):
		obstacle_area = plt.Circle((obstacles[k][0], obstacles[k][1]), obstacles[k][2], color='b', alpha=0.5)
		ax.add_artist(obstacle_area)
	figure, plt.ion()

# Setup variables
	g = [q_goal[0], q_goal[1]]
	n=q_init
	x, y = n
	theta = theta_init
	nodes = []
	nodes.append(q_init)
	node_new = q_init
	parent = []
	path = []
	curve_time = []

# Begin loop
	while(euclidean(n,g) > q_goal[2] ):
		rand = random.random()*100, random.random()*100
		t = 0.5
		v = v_0
		w = w_0
		progress = 0
		weight = 1000
		n = nodes[0]
		n_prev = nodes[0]
		for i in nodes:
			if euclidean(i, rand) < euclidean(n, rand):
				n = i
				n_prev = n
		node_new = next_node(n, rand)

		cflag = node_collision_check(node_new, obstacles)
		if cflag == 1:
			continue

# Loop for optimal acceleration selection
		for i in range(0,2500):
			progress = 0;
			x, y = n
			v = 0
			w = 0
			theta = theta_init
			a = random.uniform(-1, 1)*2
			position=x,y
			gamma = random.uniform(-1, 1)*(math.pi)/2

			while progress < epsilon:
				x_new, y_new, theta_new, v_new, w_new = car_model(x, y, theta, v, w, t, a,gamma)
				position = x,y
				position_new = x_new, y_new
				progress += euclidean(position, position_new)

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
			weight_new = euclidean(position_new, node_new)
			
			if weight_new < weight:
				weight = weight_new
				optimal_a = a
				optimal_g = gamma

		progress = 0
		a = optimal_a
		gamma = optimal_g
		x,y = n
		v = 0
		w = 0
		theta = theta_init
		time = 0

# Calculate loop using optimal acceleration
		while progress < epsilon:
			time = time + t
			position = x,y
			x_new, y_new, theta_new, v_new, w_new = car_model(x, y, theta, v, w, t,a,gamma)
			position_new = x_new, y_new
			progress += euclidean(position, position_new)

			if (v_new > v_max):
				v_new = v_max
			elif (v_new < -v_max):
				v_new = -v_max
			if (w_new > w_max):
				w_new = w_max
			elif (w_new < -w_max):
				w_new = -w_max 

			cflag = node_collision_check(position_new, obstacles)
			if cflag == 1:
				break
			#figure, plt.plot([x, x_new], [y, y_new], color = 'yellow')
			x = x_new
			y = y_new
			theta = theta_new
			n = x,y
			v = v_new
			w = w_new

			robot_plot = []
			for m in range(len(robot)):
				robot_coordinates = homogenous(x_new, y_new, theta_new, robot[m][0],robot[m][1])
				cflag3 = node_collision_check(robot_coordinates, obstacles)
				if cflag3 == 1:
					break
				robot_plot = plt.Circle((robot_coordinates[0], robot_coordinates[1]), 0.1, color='w', alpha=0.5)
				ax.add_artist(robot_plot)

		curve_time.append(time)
		if cflag == 1:
				continue
# Data set ready
		new_node = x_new, y_new
		current = new_node
		nodes.append(new_node)
		data = [new_node, n_prev, time, theta_new, v_new, w_new, a, gamma]
		parent.append(data)
		plt.pause(0.005)

	while current != q_init:
		for m in parent:
			if m[0] == current:
				current = m[1]
				path.append(m)
				break
# Write CSV
	for j in path:
		x,y = j[0]
		row = (j[0],j[1],j[2],j[3],j[5],j[6])
		writer.writerow(row)
		x2,y2 = j[1]
		figure, plt.plot([x,x2],[y,y2], color = 'red')
	
	while True:
	 	plt.pause(0.005)


#Main loop
if __name__ == '__main__':
	plot_nodes()