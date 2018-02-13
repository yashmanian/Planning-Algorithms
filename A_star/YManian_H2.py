# Yash Manian
# UID: 114965165
# A* algorithm assignment 2
# ENPM661

from collections import defaultdict
from math import sqrt
start = [772, 643, 596, 772, 643, 596]
goal = [386, 608, 609, 386, 608, 609] 
weights = [1, 10, 2, 100, 100, 100]

for i in range (0,6):
	start_node, goal_node, weight = start[i], goal[i], weights[i]
	nodes = defaultdict(dict)

	def goal_distance(a,b):
		weight = weights[i]
		x1, y1 = nodes[a]['coords']
		x2, y2 = nodes[b]['coords']
		distance = weight*sqrt(pow(x2-x1,2) + pow(y2-y1,2))
		return distance

	def get_path(current):
		total_path = [current]
		while current in parent_node:
			current = parent_node[current]
			total_path.append(current)
		return reversed(total_path)

	with open('rand1_nodes.txt') as f:
		for j, l in enumerate(f):
			node_num = j + 1
			x, y = l.split(',')
			x, y = float(x), float(y)
			nodes[node_num]['coords'] = (x, y)
			nodes[node_num]['possible_next_nodess'] = []

	with open('rand1_edges.txt') as f:
		for l in f:
			current_node, next_node, weight = l.split(',')
			current_node, next_node, weight = int(current_node), int(next_node), float(weight)
			nodes[current_node]['possible_next_nodess'].append((next_node,weight))

	nodes[42]


	visited = set()
	unvisited = set()
	unvisited.add(start_node)
	parent_node = {}

	cost_to_start = defaultdict(None)
	cost_to_start[start_node] = 0
	totalcost = defaultdict(None)
	totalcost[start_node] = goal_distance(start_node, goal_node)

	while True:
		if not len(unvisited):
			print("Failed")
			break

		top = sorted(list(unvisited), key = lambda x: totalcost[x])[:3]
		current = top[0]

		if current == goal_node:
			print(i+1)
			print("Target Reached")
			print("Cost to goatl", cost_to_start[goal_node])
			print(list(get_path(current)))
			break

		unvisited.remove(current)
		visited.add(current)

		for possible_next_nodes, cost in nodes[current]['possible_next_nodess']:
			if possible_next_nodes in visited:
				continue

			temp_cost = cost_to_start[current] + cost
			if possible_next_nodes not in unvisited:
				unvisited.add(possible_next_nodes)

			elif cost_to_start[possible_next_nodes] is not None and temp_cost >= cost_to_start[possible_next_nodes]:
				continue

			parent_node[possible_next_nodes] = current
			cost_to_start[possible_next_nodes] = temp_cost
			totalcost[possible_next_nodes] = cost_to_start[possible_next_nodes]+goal_distance(possible_next_nodes, goal_node)


