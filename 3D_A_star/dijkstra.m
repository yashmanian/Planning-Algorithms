function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.

% CMSC 828T Proj 1 Phase 1

if nargin < 4
    astar = false;
end

%% START YOUR CODE HERE %%
start = start';
goal = goal';

occupancyGrid = map{1};
x = map{2};
y = map{3};
z = map{4};
map_x = size(map{1},1);
map_y = size(map{1},2);
map_z = size(map{1},3);

current_x = 1;
current_y = 2;
current_z = 3;
parent_x = 4;
parent_y = 5;
parent_z = 6;
visited = 7;

xy_res = map{5};
z_res = map{6};
final_ws = map{7};
start_cost = 9999999;
num_expanded = 0;

% Convert coordinate to logical point

slope = [(map_x(end)-1)/(x(end)-x(1)); (map_y(end)-1)/(y(end)-y(1));(map_z(end)-1)/(z(end)-z(1))];
start_cc = [1 + slope(1)*(start(1) - x(1));1 + slope(2)*(start(2) - y(1));1 + slope(3)*(start(3) - z(1))];
goal_cc = [1 + slope(1)*(goal(1) - x(1));1 + slope(2)*(goal(2) - y(1));1 + slope(3)*(goal(3) - z(1))];

node_list = [1 + slope(1).*(final_ws(:,1) - x(1)) 1 + slope(2).*(final_ws(:,2) - y(1)) 1 + slope(3).*(final_ws(:,3) - z(1));];

% Construct Data structures
start_node = [start_cc(1) start_cc(2) start_cc(3) 0 0 0];
goal_node = [goal_cc(1) goal_cc(2) goal_cc(3) 0 0 0];

% Remove start node
nodes = [node_list(:,1) node_list(:,2) node_list(:,3) repmat([0 0 0],size(node_list,1),1)];
nodes = uint16(nodes);
I = ismember(nodes(:,1:3),start_node(1,1:3),'rows');
I = find(I);

% Collision check
C = collide(map,nodes(:,1:3));
I = C==0;
unvisited=nodes(I,:);

priority_queue = [start_node];
priority_cost = [0];
visited_nodes = [];
visited_costs = [];
PQ_size1 = 0;
PQ_size2 = 0;
val = [];


%% 
tic
while ~isempty(unvisited)
%for a = 1:27
    [u,v] = min(priority_cost);
    curr_node = priority_queue(v,:);
    curr_dist = priority_cost(v);
    priority_queue(v,:) = [];
    priority_cost(v) = [];
   

%% Get next possible nodes
    next_nodes = [];
    next_x = [curr_node(1)-1 curr_node(1) curr_node(1)+1];
    next_y = [curr_node(2)-1 curr_node(2) curr_node(2)+1];
    next_z = [curr_node(3)-1 curr_node(3) curr_node(3)+1];
    curr_node_1 = double(curr_node);
    goal_node_1 = double(goal_node);
    
    % All connected nodes
    for nx = 1:3
        for ny = 1:3
            for nz = 1:3
                if next_x(nx)<=0 || next_y(ny)<=0||next_z(nz)<=0 ||next_x(nx)>map_x || next_y(ny)>map_y ||next_z(nz)> map_z
                    continue
                end
                temp = [next_x(nx) next_y(ny) next_z(nz)];
                next_nodes = [next_nodes;temp];
            end
        end
    end
    
%% Remove current node
    I_start = ismember(next_nodes(:,1:3),curr_node(1,1:3),'rows');
    I_start = find(I_start);
    next_nodes(I_start,:) = [];

    
 %% Find indices of nodes
    c = ismember(unvisited(:,1:3),next_nodes,'rows');
    next_idx = find(c);
    next_nodes = unvisited(next_idx,:);
    node_val = double(next_nodes(:,1:3));
    
%% Get current node costs
    node_dist = [];
    for a = 1:size(next_nodes,1)
        p_idx = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(a,1:3)),2));
        if ~isempty(p_idx)
            node_dist(a,1) = priority_cost(p_idx);
        else
            node_dist(a,1) = start_cost;
        end
    end
%% Compute costs
    dist2node = sqrt((node_val(:,1)-curr_node_1(current_x)).^2 +  (node_val(:,2)-curr_node_1(current_y)).^2 +(node_val(:,3)-curr_node_1(current_z)).^2);
    heuristic = sqrt((node_val(:,1)-goal_node_1(current_x)).^2 +  (node_val(:,2)-goal_node_1(current_y)).^2 +(node_val(:,3)-goal_node_1(current_z)).^2);
    
    if astar == 1
        total_dist = curr_dist + heuristic + dist2node;
    else
        total_dist = curr_dist + dist2node;
    end
    start = [1;1;4];
goal = [7 18 3];
%% Change parents based on cost
    for m = 1:size(next_nodes,1)
        f = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(m,1:3)),2));
        if ~isempty(f) == 1
            next_nodes(m,4:6) = priority_queue(f,4:6);
        else
            continue
        end
    end
%% Cost update and parent update
    for i = 1:size(node_dist,1)
        if total_dist(i) < node_dist(i)
            node_dist(i) = total_dist(i);
            next_nodes(i,4:6) = curr_node(1:3);
        end
    end
%% Get rid of existing current node list in priority queue
    for j = 1:size(next_nodes,1)
        f = find(all(bsxfun(@eq, priority_queue(:,1:3),next_nodes(j,1:3)),2));
        if ~isempty(f)
            priority_queue(f,:) = [];
            priority_cost(f) = [];
        end
    end
    priority_queue = [priority_queue;next_nodes];
    priority_cost = [priority_cost;node_dist];
   
%% Sort priority queue and pop unvisited queue
    [priority_cost,idx] = sort(priority_cost);
    priority_queue = priority_queue(idx,:);
    visited_nodes = [visited_nodes;curr_node];
    visited_costs = [visited_costs;curr_dist];
    [~,index1] = ismember(unvisited(:,1:3),curr_node(:,1:3),'rows');
    index1 = find(index1);
    unvisited(index1,:) = [];
% Break condition
    if isequal(curr_node(1:3),goal_node(1:3)) & astar == 1
        num_expanded = size(visited_nodes,1);
        break
    end
end
toc

%% Return path
[p,q] = ismember(visited_nodes(:,1:3),goal_node(:,1:3),'rows');
q = find(q);
node = visited_nodes(q,:);
end_parent = [0 0 0];
node_parent = node(4:6);
path = [node(1:3)];

while ~isequal(end_parent,node_parent)
    path = [path;node_parent];
    node = node_parent;
    [p,q] = ismember(visited_nodes(:,1:3),node,'rows');
    q = find(q);
    node_parent = visited_nodes(q,4:6);
end
%% END YOUR CODE HERE %%

end