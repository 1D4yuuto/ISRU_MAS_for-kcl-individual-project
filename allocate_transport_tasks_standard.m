function [updated_robots, transport_allocation, transport_paths] = allocate_transport_tasks_standard(robots, resource_list, collection_results, base_pos, grid_map, weights)

% Input:
%   robots              - array of transporter robots
%   resource_list       - list of resource positions [x, y; ...]
%   collection_results  - results from collection mission
%   base_pos           - base position [x, y]
%   grid_map           - 200x200 grid map for A* path planning
%   weights            - optimization weights struct
% Output:
%   updated_robots      - robots with assigned transport tasks
%   transport_allocation - transport allocation matrix [robot_id, resource_id, cost]
%   transport_paths     - struct array with path data and robot mapping

    num_robots = length(robots);
    num_resources = size(resource_list, 1);
    
    fprintf('  Allocating %d transport tasks to %d transporter robots using Standard A*...\n', ...
            num_resources, num_robots);
    
    % Initialize outputs
    transport_allocation = [];
    transport_paths = struct('robot_id', {}, 'resource_id', {}, 'outbound_path', {}, 'return_path', {});
    assigned_resources = false(1, num_resources);
    
    % All transporter robots start from base
    robot_positions = repmat(base_pos, num_robots, 1);
    robot_available_times = zeros(num_robots, 1);  % When each robot becomes available
    
    % Transport allocation rounds - continue until all resources transported
    round_num = 1;
    
    while any(~assigned_resources)
        fprintf('    === Transport Round %d ===\n', round_num);
        
        available_resources = find(~assigned_resources);
        if isempty(available_resources)
            break;
        end
        
        fprintf('    Available resources for transport: %s\n', sprintf('%d ', available_resources));
        
        % Calculate cost matrix for transport tasks (round trip costs) using Standard A*
        cost_matrix = calculate_transport_cost_matrix_standard(robot_positions, resource_list, ...
                                                              available_resources, base_pos, ...
                                                              grid_map, robots, weights);
        
        % Assign transport tasks in this round
        num_assignments_this_round = min(num_robots, length(available_resources));
        
        for assignment = 1:num_assignments_this_round
            % Find minimum cost assignment
            [min_cost, min_idx] = min(cost_matrix(:));
            if isinf(min_cost)
                fprintf('    Warning: No feasible transport paths found\n');
                break;
            end
            
            [robot_idx, resource_idx] = ind2sub(size(cost_matrix), min_idx);
            actual_resource_id = available_resources(resource_idx);
            resource_pos = resource_list(actual_resource_id, :);
            
            try
                % *** 关键修改：使用标准A*规划路径 ***
                % Plan outbound path: current position -> resource
                current_pos = robot_positions(robot_idx, :);
                outbound_path = astar_pathfinding_standard(current_pos, resource_pos, grid_map, ...
                                                          robots(robot_idx), weights);
                
                % Plan return path: resource -> base
                return_path = astar_pathfinding_standard(resource_pos, base_pos, grid_map, ...
                                                        robots(robot_idx), weights);
                
                % Calculate total round-trip cost
                total_cost = outbound_path.total_cost + return_path.total_cost;
                
                % Record allocation
                transport_allocation(end+1, :) = [robots(robot_idx).id, actual_resource_id, total_cost];
                
                % Store transport paths with robot and resource mapping
                path_record = struct();
                path_record.robot_id = robots(robot_idx).id;
                path_record.resource_id = actual_resource_id;
                path_record.outbound_path = outbound_path;
                path_record.return_path = return_path;
                transport_paths(end+1) = path_record;
                
                % Update robot assignment
                robots(robot_idx).assigned_resources(end+1) = actual_resource_id;
                robots(robot_idx).current_status = 'assigned_transport';
                
                % Update robot position to base (after completing round trip)
                robot_positions(robot_idx, :) = base_pos;
                
                % Update robot available time (time to complete this round trip)
                trip_time = outbound_path.total_time + return_path.total_time;
                robot_available_times(robot_idx) = robot_available_times(robot_idx) + trip_time;
                
                % Mark resource as transported
                assigned_resources(actual_resource_id) = true;
                
                fprintf('    Robot %d -> Resource %d (Round-trip cost: %.2f, Time: %.1fs)\n', ...
                        robots(robot_idx).id, actual_resource_id, total_cost, trip_time);
                
                % Remove this assignment from current round
                cost_matrix(robot_idx, :) = inf;
                cost_matrix(:, resource_idx) = inf;
                
            catch ME
                fprintf('    Warning: Failed to plan transport for Robot %d to Resource %d: %s\n', ...
                        robots(robot_idx).id, actual_resource_id, ME.message);
                cost_matrix(robot_idx, resource_idx) = inf;
                continue;
            end
        end
        
        fprintf('    Round %d completed: %d transport assignments made\n', round_num, num_assignments_this_round);
        round_num = round_num + 1;
        
        % Safety check
        if round_num > num_resources
            fprintf('    Warning: Maximum rounds exceeded\n');
            break;
        end
    end
    
    % Final summary
    total_assignments = size(transport_allocation, 1);
    total_cost = sum(transport_allocation(:, 3));
    
    fprintf('  === Transport Task Allocation Summary (Standard A*) ===\n');
    fprintf('    Total transport assignments: %d/%d resources\n', total_assignments, num_resources);
    fprintf('    Total transport cost: %.2f\n', total_cost);
    fprintf('    Average cost per transport: %.2f\n', total_cost / total_assignments);
    fprintf('    Generated %d transport path records\n', length(transport_paths));
    
    % Update robot final status
    for i = 1:num_robots
        robots(i).total_time = robot_available_times(i);
        robots(i).position = base_pos;  % All end at base
    end
    
    updated_robots = robots;
    
end

%% Calculate transport cost matrix using Standard A* (round-trip costs, ignoring terrain)
function cost_matrix = calculate_transport_cost_matrix_standard(robot_positions, resource_list, ...
                                                               available_resources, base_pos, ...
                                                               grid_map, robots, weights)
% Calculate round-trip transport costs using Standard A* for all robot-resource combinations
    
    num_robots = size(robot_positions, 1);
    num_available_resources = length(available_resources);
    cost_matrix = inf(num_robots, num_available_resources);
    
    fprintf('    Calculating Standard A* transport cost matrix (%dx%d)...\n', ...
            num_robots, num_available_resources);
    
    for i = 1:num_robots
        robot_pos = robot_positions(i, :);
        
        for j = 1:num_available_resources
            resource_id = available_resources(j);
            resource_pos = resource_list(resource_id, :);
            
            try
                % *** 关键修改：使用标准A*计算往返路径 ***
                % Calculate outbound trip: current position -> resource
                outbound_path = astar_pathfinding_standard(robot_pos, resource_pos, grid_map, ...
                                                          robots(i), weights);
                
                % Calculate return trip: resource -> base
                return_path = astar_pathfinding_standard(resource_pos, base_pos, grid_map, ...
                                                        robots(i), weights);
                
                % Total round-trip cost
                total_cost = outbound_path.total_cost + return_path.total_cost;
                cost_matrix(i, j) = total_cost;
                
            catch ME
                % If no path found, set cost to infinity
                cost_matrix(i, j) = inf;
            end
        end
    end
    
    % Display cost summary
    finite_costs = cost_matrix(isfinite(cost_matrix));
    if ~isempty(finite_costs)
        fprintf('    Standard A* transport costs: min=%.2f, max=%.2f, avg=%.2f\n', ...
                min(finite_costs), max(finite_costs), mean(finite_costs));
    end
    
end