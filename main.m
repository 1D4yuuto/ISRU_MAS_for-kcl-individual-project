clc;
clear;

%% === System Parameter Settings ===
OBSTACLE_THRESHOLD = 0.2;       % Obstacle coverage threshold (20%)
ENERGY_TERRAIN_THRESHOLD = 0.4; % Energy terrain coverage threshold (20%)
OPTIMIZATION_MODE = 'distance';  % Optimization mode: 'distance', 'time', 'energy', 'hybrid'

fprintf('=== Lunar ISRU Multi-Agent System ===\n');
fprintf('Optimization mode: %s\n', OPTIMIZATION_MODE);

%% Step 1: Generate Map
fprintf('\n=== Step 1: Generate Map ===\n');
[map, ~, color_idx_map, ~] = map_generate();%function1

% Use improved grid generation (considering threshold)
grid_map = generateGridMap(map, 50, OBSTACLE_THRESHOLD, ENERGY_TERRAIN_THRESHOLD);%function2
color_idx_map_coarse = generate_color_map(grid_map);%function3

% Get key positions
[base_y, base_x] = find(grid_map == 8);
base_pos = round([mean(base_x), mean(base_y)]); % Calculate center of base area
[res_y, res_x] = find(grid_map == 9);
raw_resources = [res_x, res_y];
resource_list = cluster_resources(raw_resources);%function4

fprintf('Base position: [%.1f, %.1f], Resource points: %d\n', base_pos(1), base_pos(2), size(resource_list, 1));
display_maps(map, color_idx_map, grid_map, color_idx_map_coarse, base_pos, resource_list);%function4.5

%% Step 2: Robot Initialization
fprintf('\n=== Step 2: Robot Initialization ===\n');
num_collectors = 5;
num_transporters = 3;

[collector_robots, transporter_robots] = init_robots(base_pos, num_collectors, num_transporters);%function5

%% Step 3: Collection Task Allocation (with A* Path Planning)
fprintf('\n=== Step 3: Collection Task Allocation ===\n');
optimization_weights = get_optimization_weights(OPTIMIZATION_MODE);
[collector_robots, collection_allocation, collection_paths] = allocate_collection_tasks(...
    collector_robots, resource_list, grid_map, optimization_weights);
%print_collection_allocation(collection_allocation);

%% Step 4: Collection Task Execution 
fprintf('\n=== Step 4: Collection Task Execution ===\n');
[collector_robots, collection_results] = execute_collection_mission(...
    collector_robots, collection_paths, base_pos, grid_map);
%print_collection_results(collection_results);

%% Step 5: Transport Task Allocation
fprintf('\n=== Step 5: Transport Task Allocation ===\n');
[transporter_robots, transport_allocation, transport_paths] = allocate_transport_tasks(...
    transporter_robots, resource_list, collection_results, base_pos, grid_map, optimization_weights);%function11
%print_transport_allocation(transport_allocation);%function12

%% Step 6: Transport Task Execution
fprintf('\n=== Step 6: Transport Task Execution ===\n');
[transporter_robots, transport_results] = execute_transport_mission(...
    transporter_robots, transport_paths, base_pos, grid_map);%function13
%print_transport_results(transport_results);%function14
