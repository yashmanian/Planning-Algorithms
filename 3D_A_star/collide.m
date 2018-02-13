function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

% CMSC 828T Proj 1 Phase 1

% If the map is empty, output empty vector else, compute
C = [];
if isempty(map)
    C = [];
else
    %% START YOUR CODE HERE %%
    occupancyGrid = map{1};
    for i = 1:size(points,1)
        if occupancyGrid(points(i,1),points(i,2), points(i,3)) == 0
            C = [C; 0];
        else
            C = [C;1];
        end
    %% END YOUR CODE HERE %%
end
end
