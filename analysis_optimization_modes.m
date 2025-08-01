% analysis_optimization_modes.m

clc; clear;

fprintf('=== Optimization Modes Comparison Analysis ===\n');

%% Initialize the environment (silent mode)
[map, ~, ~, ~] = map_generate();
grid_map = generateGridMap_silent(map, 50, 0.2, 0.4);
[base_y, base_x] = find(grid_map == 8);
base_pos = round([mean(base_x), mean(base_y)]);
[res_y, res_x] = find(grid_map == 9);
resource_list = cluster_resources_silent([res_x, res_y]);

fprintf('Environment: Base[%.1f,%.1f], Resources: %d\n', ...
        base_pos(1), base_pos(2), size(resource_list, 1));

%% Experiment 1: Comparison of optimization modes
fprintf('\n=== Experiment 1: Optimization Mode Comparison ===\n');
modes = {'distance', 'time', 'energy', 'hybrid'};
results = struct();

for i = 1:length(modes)
    fprintf('Testing %s mode... ', upper(modes{i}));
    
   
    weights = get_optimization_weights(modes{i});
    [collector_robots, transporter_robots] = init_robots(base_pos, 5, 3);
    
    % Execute the task and save the path information (silent mode)
    [collector_robots, collection_allocation, collection_paths] = allocate_collection_tasks(...
        collector_robots, resource_list, grid_map, weights);
    [collector_robots, collection_results] = execute_collection_mission_silent(...
        collector_robots, collection_paths, base_pos, grid_map);
    [transporter_robots, transport_allocation, transport_paths] = allocate_transport_tasks(...
        transporter_robots, resource_list, collection_results, base_pos, grid_map, weights);
    [transporter_robots, transport_results] = execute_transport_mission_silent(...
        transporter_robots, transport_paths, base_pos, grid_map);
    
    % Calculate actual movement time
    collection_time = 0;
    for j = 1:length(collection_paths)
        collection_time = collection_time + collection_paths(j).path_result.total_time;
    end
    
    transport_time = 0;
    for j = 1:length(transport_paths)
        transport_time = transport_time + transport_paths(j).outbound_path.total_time + transport_paths(j).return_path.total_time;
    end
    
    % The total time is the maximum value of parallel execution
    total_time = max(collection_time, transport_time);
    total_energy = sum([collector_robots.total_energy]) + sum([transporter_robots.total_energy]);
    completed = size(transport_allocation, 1);
    
    % Storing results
    safety_analysis = analyze_mission_safety(collection_paths, transport_paths, grid_map);
    
    % 
    results.(modes{i}) = struct();
    results.(modes{i}).time = total_time;
    results.(modes{i}).energy = total_energy;
    results.(modes{i}).completed = completed;
    results.(modes{i}).safety = safety_analysis.overall_mission_success;
    results.(modes{i}).collection_paths = collection_paths;
    results.(modes{i}).transport_paths = transport_paths;
    
    fprintf('Time: %.1fs, Energy: %.1fJ, Safety: %.1f%%, Completed: %d/%d\n', ...
            total_time, total_energy, safety_analysis.overall_mission_success*100, completed, size(resource_list, 1));
    
    % Storing benchmark results
    if strcmp(modes{i}, 'distance')
        baseline_time = total_time;
        baseline_energy = total_energy;
        baseline_safety = safety_analysis.overall_mission_success;
    end
end

%% Comparative analysis of results
fprintf('\n=== Optimization Modes Performance Summary ===\n');
fprintf('%-12s %-12s %-12s %-12s %-12s %-12s\n', ...
        'Mode', 'Time(h)', 'Energy(MJ)', 'Safety(%)', 'Time_Diff', 'Energy_Diff');
fprintf('%-12s %-12s %-12s %-12s %-12s %-12s\n', ...
        '----', '-------', '----------', '--------', '---------', '-----------');

for i = 1:length(modes)
    mode_data = results.(modes{i});
    time_diff = (mode_data.time - baseline_time) / baseline_time * 100;
    energy_diff = (mode_data.energy - baseline_energy) / baseline_energy * 100;
    
    fprintf('%-12s %-12.1f %-12.3f %-12.1f %-12.1f%% %-12.1f%%\n', ...
            upper(modes{i}), mode_data.time/3600, mode_data.energy/1000000, ...
            mode_data.safety*100, time_diff, energy_diff);
end

fprintf('\n=== Optimization Modes Analysis Complete ===\n');

%% Security Analysis Function
function safety_analysis = analyze_mission_safety(collection_paths, transport_paths, grid_map)
    
    safety_analysis = struct();
    
    % Analyze and collect mission security
    collection_safety = [];
    for i = 1:length(collection_paths)
        path = collection_paths(i).path_result.path;
        safety = calculate_path_safety(path, grid_map);
        collection_safety(i) = safety.success_probability;
    end
    
    % Analyze the safety of transportation tasks
    transport_safety = [];
    for i = 1:length(transport_paths)
        % Round-trip path security
        outbound_safety = calculate_path_safety(transport_paths(i).outbound_path.path, grid_map);
        return_safety = calculate_path_safety(transport_paths(i).return_path.path, grid_map);
        
        % Overall success probability of round-trip mission
        round_trip_success = outbound_safety.success_probability * return_safety.success_probability;
        transport_safety(i) = round_trip_success;
    end
    
    % Overall mission success probability (all subtasks succeed)
    overall_collection_success = prod(collection_safety);
    overall_transport_success = prod(transport_safety);
    overall_mission_success = overall_collection_success * overall_transport_success;
    
    safety_analysis.collection_safety = collection_safety;
    safety_analysis.transport_safety = transport_safety;
    safety_analysis.overall_collection_success = overall_collection_success;
    safety_analysis.overall_transport_success = overall_transport_success;
    safety_analysis.overall_mission_success = overall_mission_success;
end

%% Path safety calculation function
function safety_cost = calculate_path_safety(path, grid_map)
    total_risk = 0;
    total_distance = 0;
    
    for i = 1:size(path, 1) - 1
        current = path(i, :);
        next = path(i + 1, :);
        
        % Calculation distance (km)
        segment_distance = norm(next - current) * 0.05; % 50m per grid -> km
        total_distance = total_distance + segment_distance;
        
        % Get terrain risk
        terrain_val = grid_map(next(2), next(1));
        risk_rate = get_terrain_risk(terrain_val);
        
        % Cumulative risk
        segment_risk = risk_rate * segment_distance;
        total_risk = total_risk + segment_risk;
    end
    
    % Calculate the overall failure probability of the path
    failure_probability = 1 - exp(-total_risk);
    success_probability = 1 - failure_probability;
    
    safety_cost = struct();
    safety_cost.total_risk = total_risk;
    safety_cost.failure_probability = failure_probability;
    safety_cost.success_probability = success_probability;
    safety_cost.total_distance_km = total_distance;
end

%% terrain risk function
function risk_factor = get_terrain_risk(terrain_value)
    % error rate(per km)
    switch terrain_value
        case 0
            risk_factor = inf;  % obstacles
        case 1
            risk_factor = 0.001;  % normal - 0.1%error
        case 3
            risk_factor = 0.015;  % 3x - 1.5%error
        case 4
            risk_factor = 0.035;  % 4x - 3.5%error
        case {8, 9}
            risk_factor = 0.001;  % base and resources - safety
        otherwise
            risk_factor = 0.001;
    end
end