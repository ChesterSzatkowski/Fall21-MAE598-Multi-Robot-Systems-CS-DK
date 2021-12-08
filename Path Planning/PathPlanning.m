%% Creating a map

warehouseMap = readmatrix('map.txt'); % Matrix where 1 represents an obstacle and 0 represents a free space

map = binaryOccupancyMap(warehouseMap,1); % 1 cell per meter
% show(map) 

robotRadius = 0.2; % robots are assumed to be circles of radius 0.2 meters

mapInflated = copy(map);
inflate(mapInflated,robotRadius); % to account for robot's dimension and ensure no collisions occur

% show(mapInflated)

prm = mobileRobotPRM; % define a path planner

prm.Map = mapInflated; % assign map to path planner
prm.NumNodes = 150; % number of randomly sampled nodes in free space
prm.ConnectionDistance = 10; % maximum line that connects any two nodes; increasing can increase computation time

startLocation = [20, 38];
endLocation = [20, 2];

path = findpath(prm, startLocation, endLocation); % solve the path

show(prm)
