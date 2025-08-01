% analysis_algorithm_comparison.m 


clc; clear;

fprintf('=== Algorithm Comparison Analysis ===\n');

%% Initialize silent
[map, ~, ~, ~] = map_generate();
grid_map = generateGridMap_silent(map, 50, 0.2, 0.4);
[base_y, base_x] = find(grid_map == 8);
base_pos = round([mean(base_x), mean(base_y)]);
[res_y, res_x] = find(grid_map == 9);
resource_list = cluster_resources_silent([res_x, res_y]);

fprintf('Environment: Base[%.1f,%.1f], Resources: %d\n', ...
        base_pos(1), base_pos(2), size(resource_list, 1));

%% comparison
fprintf('\n=== Algorithm Comparison Experiment ===\n');

% define configuration
algorithms = {
    {'Standard A* (No Terrain)', struct('distance', 1.0, 'time', 0.0, 'energy', 0.0), 'standard'},
    {'Terrain-aware A*', struct('distance', 0.3, 'time', 0.4, 'energy', 0.3), 'terrain'}
};

algorithm_results = [];
algorithm_safety = [];
algorithm_paths = {};

for i = 1:length(algorithms)
    alg_name = algorithms{i}{1};
    alg_weights = algorithms{i}{2};
    alg_type = algorithms{i}{3};
    
    fprintf('%s... ', alg_name);
    [collectors, transporters] = init_robots(base_pos, 5, 3);
    
    % choose pathfinding algorithms
    if strcmp(alg_type, 'standard')
        % standard A
        [collectors, coll_alloc, coll_paths] = allocate_collection_tasks_standard(...
            collectors, resource_list, grid_map, alg_weights);
        [collectors, ~] = execute_collection_mission_silent(collectors, coll_paths, base_pos, grid_map);
        [transporters, trans_alloc, trans_paths] = allocate_transport_tasks_standard(...
            transporters, resource_list, [], base_pos, grid_map, alg_weights);
        [transporters, ~] = execute_transport_mission_silent(transporters, trans_paths, base_pos, grid_map);
    else
        % terrain A
        [collectors, coll_alloc, coll_paths] = allocate_collection_tasks(...
            collectors, resource_list, grid_map, alg_weights);
        [collectors, ~] = execute_collection_mission_silent(collectors, coll_paths, base_pos, grid_map);
        [transporters, trans_alloc, trans_paths] = allocate_transport_tasks(...
            transporters, resource_list, [], base_pos, grid_map, alg_weights);
        [transporters, ~] = execute_transport_mission_silent(transporters, trans_paths, base_pos, grid_map);
    end
    
    % calculate actual time
    coll_time = 0;
    for j = 1:length(coll_paths)
        coll_time = coll_time + coll_paths(j).path_result.total_time;
    end
    
    trans_time = 0;
    for j = 1:length(trans_paths)
        trans_time = trans_time + trans_paths(j).outbound_path.total_time + trans_paths(j).return_path.total_time;
    end
    
    total_time = max(coll_time, trans_time);
    total_energy = sum([collectors.total_energy]) + sum([transporters.total_energy]);
    
    % safety analysis
    safety_analysis = analyze_mission_safety(coll_paths, trans_paths, grid_map);
    
    % Storing results
    algorithm_results(i,:) = [total_time, total_energy, sum(coll_alloc(:,3)), sum(trans_alloc(:,3))];
    algorithm_safety(i) = safety_analysis.overall_mission_success;
    algorithm_paths{i} = {coll_paths, trans_paths};
    
    fprintf('Time: %.1fs, Energy: %.1fJ, Safety: %.1f%%\n', ...
            total_time, total_energy, safety_analysis.overall_mission_success*100);
end

%% risk analysis
fprintf('\n=== Detailed Path Risk Analysis ===\n');
for i = 1:length(algorithms)
    fprintf('\n%s Path Analysis:\n', algorithms{i}{1});
    coll_paths = algorithm_paths{i}{1};
    
    fprintf('Collection Paths Risk Assessment:\n');
    for j = 1:length(coll_paths)
        path = coll_paths(j).path_result.path;
        safety = calculate_path_safety(path, grid_map);
        terrain_analysis = analyze_path_terrain(path, grid_map);
        
        fprintf('  Robot %d -> Resource %d: Success=%.1f%%, Distance=%.2fkm, Risk Terrain=%.1f%%\n', ...
                coll_paths(j).robot_id, coll_paths(j).resource_id, ...
                safety.success_probability*100, safety.total_distance_km, ...
                terrain_analysis.high_risk_percentage);
    end
end

%% results
fprintf('\n=== Algorithm Performance Comparison ===\n');
fprintf('%-25s %-12s %-12s %-12s %-12s\n', 'Algorithm', 'Time(h)', 'Energy(MJ)', 'Safety(%)', 'Risk Score');
fprintf('%-25s %-12s %-12s %-12s %-12s\n', '---------', '-------', '----------', '--------', '----------');

standard_idx = 1;  % Standard A*
terrain_idx = 2;   % Terrain-aware A*

for i = 1:length(algorithms)
    risk_score = calculate_composite_risk_score(algorithm_results(i,1), algorithm_results(i,2), algorithm_safety(i));
    fprintf('%-25s %-12.1f %-12.3f %-12.1f %-12.3f\n', ...
            algorithms{i}{1}, algorithm_results(i,1)/3600, algorithm_results(i,2)/1000000, ...
            algorithm_safety(i)*100, risk_score);
end


time_improve = (algorithm_results(standard_idx,1) - algorithm_results(terrain_idx,1)) / algorithm_results(standard_idx,1) * 100;
energy_improve = (algorithm_results(standard_idx,2) - algorithm_results(terrain_idx,2)) / algorithm_results(standard_idx,2) * 100;
safety_improve = (algorithm_safety(terrain_idx) - algorithm_safety(standard_idx)) * 100;

fprintf('\n=== Terrain-aware A* vs Standard A* Analysis ===\n');
if time_improve > 0
    fprintf('Time Performance: %.1f%% faster (BETTER)\n', time_improve);
else
    fprintf('Time Performance: %.1f%% slower (trade-off for safety)\n', abs(time_improve));
end

if energy_improve > 0
    fprintf('Energy Performance: %.1f%% better\n', energy_improve);
else
    fprintf('Energy Performance: %.1f%% worse (trade-off for safety)\n', abs(energy_improve));
end

fprintf('Safety Improvement: +%.1f%% (%.1f%% -> %.1f%%)\n', ...
        safety_improve, algorithm_safety(standard_idx)*100, algorithm_safety(terrain_idx)*100);

% comrehensive analysis
overall_benefit = calculate_overall_benefit(time_improve, energy_improve, safety_improve);
fprintf('Overall Benefit Score: %.2f (considering safety value)\n', overall_benefit);

fprintf('\n=== Algorithm Comparison Analysis Complete ===\n');

%% define support function

function safety_analysis = analyze_mission_safety(collection_paths, transport_paths, grid_map)
    
    safety_analysis = struct();
   
    collection_safety = [];
    for i = 1:length(collection_paths)
        path = collection_paths(i).path_result.path;
        safety = calculate_path_safety(path, grid_map);
        collection_safety(i) = safety.success_probability;
    end
    
    transport_safety = [];
    for i = 1:length(transport_paths)
        outbound_safety = calculate_path_safety(transport_paths(i).outbound_path.path, grid_map);
        return_safety = calculate_path_safety(transport_paths(i).return_path.path, grid_map);
        round_trip_success = outbound_safety.success_probability * return_safety.success_probability;
        transport_safety(i) = round_trip_success;
    end
    
    overall_collection_success = prod(collection_safety);
    overall_transport_success = prod(transport_safety);
    overall_mission_success = overall_collection_success * overall_transport_success;
    
    safety_analysis.collection_safety = collection_safety;
    safety_analysis.transport_safety = transport_safety;
    safety_analysis.overall_collection_success = overall_collection_success;
    safety_analysis.overall_transport_success = overall_transport_success;
    safety_analysis.overall_mission_success = overall_mission_success;
end

function safety_cost = calculate_path_safety(path, grid_map)

    total_risk = 0;
    total_distance = 0;
    
    for i = 1:size(path, 1) - 1
        current = path(i, :);
        next = path(i + 1, :);
        
        segment_distance = norm(next - current) * 0.05; % km
        total_distance = total_distance + segment_distance;
        
        terrain_val = grid_map(next(2), next(1));
        risk_rate = get_terrain_risk(terrain_val);
        segment_risk = risk_rate * segment_distance;
        total_risk = total_risk + segment_risk;
    end
    
    failure_probability = 1 - exp(-total_risk);
    success_probability = 1 - failure_probability;
    
    safety_cost = struct();
    safety_cost.total_risk = total_risk;
    safety_cost.failure_probability = failure_probability;
    safety_cost.success_probability = success_probability;
    safety_cost.total_distance_km = total_distance;
end

function risk_factor = get_terrain_risk(terrain_value)
    switch terrain_value
        case 0
            risk_factor = inf;
        case 1
            risk_factor = 0.001;  % 0.1% error
        case 3
            risk_factor = 0.015;  % 1.5%
        case 4
            risk_factor = 0.035;  % 3.5%
        case {8, 9}
            risk_factor = 0.001;
        otherwise
            risk_factor = 0.001;
    end
end

function terrain_analysis = analyze_path_terrain(path, grid_map)

    terrain_counts = [0, 0, 0, 0, 0, 0]; % [obstacle, normal, 3x, 4x, base, resource]
    
    for i = 1:size(path, 1)
        terrain_val = grid_map(path(i,2), path(i,1));
        switch terrain_val
            case 0, terrain_counts(1) = terrain_counts(1) + 1;
            case 1, terrain_counts(2) = terrain_counts(2) + 1;
            case 3, terrain_counts(3) = terrain_counts(3) + 1;
            case 4, terrain_counts(4) = terrain_counts(4) + 1;
            case 8, terrain_counts(5) = terrain_counts(5) + 1;
            case 9, terrain_counts(6) = terrain_counts(6) + 1;
        end
    end
    
    total_points = sum(terrain_counts);
    high_risk_percentage = (terrain_counts(3) + terrain_counts(4)) / total_points * 100;
    
    terrain_analysis = struct();
    terrain_analysis.terrain_counts = terrain_counts;
    terrain_analysis.high_risk_percentage = high_risk_percentage;
    terrain_analysis.total_points = total_points;
end

function risk_score = calculate_composite_risk_score(time, energy, safety)
    % Overall risk score (lower is better)
    % Normalized and weighted
    time_norm = time / 500000; % Based on a typical timeframe
    energy_norm = energy / 800000; % Based on typical energy consumption range
    safety_norm = 1 - safety; 
    
    risk_score = 0.3 * time_norm + 0.3 * energy_norm + 0.4 * safety_norm;
end

function benefit = calculate_overall_benefit(time_change, energy_change, safety_change)
    % Calculate comprehensive benefits (taking security into account)
    % Negative changes in time and energy consumption are costs, positive changes in safety are benefits
    time_penalty = max(0, -time_change) * 0.2;
    energy_penalty = max(0, -energy_change) * 0.2;
    safety_benefit = safety_change * 0.6; % Security has the highest weight
    
    benefit = safety_benefit - time_penalty - energy_penalty;
end