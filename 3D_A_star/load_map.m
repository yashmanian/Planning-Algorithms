function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  an obstacle.

% CMSC 828T Proj 1 Phase 1

% Output structure: 
    % Map is a cell array containing the following:
    %   --> map{1} contains a 3D logical occupancy grid
    %   --> map{2}, Map{3} and Map{4} store the discretized axes values
    %       corresponding to the X, Y and Z axes respectively
    %   --> map{5} and map{6} store the xy_res and z_res respectively

% filename = ('map0.txt');
% xy_res = 0.5;
% z_res = 0.5;
% margin = 0.5;

% Open file, read data, close file. Comments in file marked by #
%fileID = fopen(filename);
fileID = fopen(filename);
fileDat = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f','CommentStyle','#');
fclose(fileID);

%% START YOUR CODE HERE %%
x_init = fileDat{2};
y_init = fileDat{3};
z_init = fileDat{4};
x_end = fileDat{5};
y_end = fileDat{6};
z_end = fileDat{7};
R = fileDat{8};
G = fileDat{9};
B = fileDat{10};
%%
% Boundary constraints
boundary = [x_init(1) y_init(1) z_init(1); x_end(1) y_end(1) z_end(1)];
block_lim = [x_init(2:end) y_init(2:end) z_init(2:end) x_end(2:end) y_end(2:end) z_end(2:end) R(2:end) G(2:end) B(2:end)];

%%
x = x_init(1):xy_res:x_end(1);
y = y_init(1):xy_res:y_end(1);
z = z_init(1):z_res:z_end(1);
x_dim = size(x,2);
y_dim = size(y,2);
z_dim = size(z,2);

%%
% All workspace points
workspace = [];
n = 1;

for x_ws = boundary(1,1):xy_res:boundary(2,1) 
    for y_ws = boundary(1,2):xy_res:boundary(2,2) 
        temp = [repmat(x_ws,size(z,2),1), repmat(y_ws,size(z,2),1), z'];
        workspace = [workspace;temp];
    end
end


%% All block points
blocks = [];
for a = 1:size(block_lim,1)
    curr_block = block_lim(a,:);
    blockX = block_lim(a,1):.2:block_lim(a,4);
    blockY = block_lim(a,2):.2:block_lim(a,5);
    blockZ = block_lim(a,3):.2:block_lim(a,6);
    for x_block = block_lim(a,1):0.2:block_lim(a,4)
        for y_block = block_lim(a,2):0.2:block_lim(a,5)
            temp = [repmat(x_block,size(blockZ,2),1), repmat(y_block,size(blockZ,2),1), blockZ'];
            blocks = [blocks;temp];
        end
    end
end

%% Logical grid
occupancyGrid = zeros(size(x,2), size(y,2), size(z,2));

for a = 1:size(blocks,1)
    for b = 1:size(workspace,1)
        dx = blocks(a,1)-workspace(b,1); 
        dy = blocks(a,2)-workspace(b,2); 
        dz = blocks(a,3)-workspace(b,3);
        dist = (dx.*dx +dy.*dy +dz.*dz)^.5;
        if dist < margin
            xGrid = (workspace(b,1)-x(1))/xy_res + 1;
            yGrid = (workspace(b,2)-y(1))/xy_res + 1;
            zGrid = (workspace(b,3)-z(1))/z_res + 1;
            occupancyGrid(int8(xGrid),int8(yGrid),int8(zGrid)) = 1;
        end
    end
end

%% Output
map = cell(1,7);
map{1} = occupancyGrid;
map{2} = x;
map{3} = y;
map{4} = z;
map{5} = xy_res;
map{6} = z_res;
map{7} = workspace;

%% END YOUR CODE HERE %%
end
