clc;
clear;

%% === Enhanced Terrain System Parameter Settings ===
OBSTACLE_THRESHOLD = 0.2;       % Obstacle coverage threshold (20%)
ENERGY_TERRAIN_THRESHOLD = 0.4; % Energy terrain coverage threshold (40%)
OPTIMIZATION_MODE = 'distance';  % Optimization mode: 'distance', 'time', 'energy', 'hybrid'

fprintf('=== Lunar ISRU Multi-Agent System (Enhanced Terrain) ===\n');
fprintf('Optimization mode: %s\n', OPTIMIZATION_MODE);

%% Step 1: Generate Enhanced Map
fprintf('\n=== Step 1: Generate Enhanced Map ===\n');
[map, grid_map, color_idx_map, color_idx_map_coarse] = map_generate_enhanced();

% Get key positions
[base_y, base_x] = find(grid_map == 8);
base_pos = round([mean(base_x), mean(base_y)]); % Calculate center of base area
[res_y, res_x] = find(grid_map == 9);
raw_resources = [res_x, res_y];
resource_list = cluster_resources(raw_resources);

fprintf('Enhanced Map - Base position: [%.1f, %.1f], Resource points: %d\n', ...
        base_pos(1), base_pos(2), size(resource_list, 1));

% Display enhanced map
display_maps(map, color_idx_map, grid_map, color_idx_map_coarse, base_pos, resource_list);

%% Step 2: Robot Initialization
fprintf('\n=== Step 2: Robot Initialization ===\n');
num_collectors = 5;
num_transporters = 3;

[collector_robots, transporter_robots] = init_robots(base_pos, num_collectors, num_transporters);

%% Step 3: Collection Task Allocation (with A* Path Planning)
fprintf('\n=== Step 3: Collection Task Allocation (Enhanced Terrain) ===\n');
optimization_weights = get_optimization_weights(OPTIMIZATION_MODE);
[collector_robots, collection_allocation, collection_paths] = allocate_collection_tasks(...
    collector_robots, resource_list, grid_map, optimization_weights);

%% Step 4: Collection Task Execution 
fprintf('\n=== Step 4: Collection Task Execution (Enhanced Terrain) ===\n');
[collector_robots, collection_results] = execute_collection_mission(...
    collector_robots, collection_paths, base_pos, grid_map);

%% Step 5: Transport Task Allocation
fprintf('\n=== Step 5: Transport Task Allocation (Enhanced Terrain) ===\n');
[transporter_robots, transport_allocation, transport_paths] = allocate_transport_tasks(...
    transporter_robots, resource_list, collection_results, base_pos, grid_map, optimization_weights);

%% Step 6: Transport Task Execution
fprintf('\n=== Step 6: Transport Task Execution (Enhanced Terrain) ===\n');
[transporter_robots, transport_results] = execute_transport_mission(...
    transporter_robots, transport_paths, base_pos, grid_map);

fprintf('\n=== Enhanced Terrain Mission Completed ===\n');

%% Display Enhanced Results Summary
fprintf('\n=== Enhanced Terrain Results Summary ===\n');
collection_time = 0;
for i = 1:length(collection_paths)
    collection_time = collection_time + collection_paths(i).path_result.total_time;
end

transport_time = 0;
for i = 1:length(transport_paths)
    transport_time = transport_time + transport_paths(i).outbound_path.total_time + transport_paths(i).return_path.total_time;
end

total_time = max(collection_time, transport_time);
total_energy = sum([collector_robots.total_energy]) + sum([transporter_robots.total_energy]);

fprintf('Enhanced Terrain Performance:\n');
fprintf('  Total Time: %.1fs (%.1f hours)\n', total_time, total_time/3600);
fprintf('  Total Energy: %.1fJ (%.2f MJ)\n', total_energy, total_energy/1000000);
fprintf('  Collection Tasks: %d/%d completed\n', size(collection_allocation,1), size(resource_list,1));
fprintf('  Transport Tasks: %d/%d completed\n', size(transport_allocation,1), size(resource_list,1));