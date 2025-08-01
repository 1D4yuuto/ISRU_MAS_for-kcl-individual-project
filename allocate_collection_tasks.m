function [updated_robots, allocation, collection_paths] = allocate_collection_tasks(robots, resource_list, grid_map, weights)
% Allocate collection tasks to collector robots using A* path planning
% Multi-round allocation until all resources are assigned
% Input:
%   robots       - array of collector robots
%   resource_list - list of resource positions [x, y; ...]
%   grid_map     - 200x200 grid map for A* path planning
%   weights      - optimization weights struct
% Output:
%   updated_robots     - robots with assigned tasks and updated positions
%   allocation         - task allocation matrix [robot_id, resource_id, cost]
%   collection_paths   - struct array with path data and robot mapping

    num_robots = length(robots);
    num_resources = size(resource_list, 1);
    
    fprintf('  Allocating %d resources to %d collector robots using A* algorithm...\n', ...
            num_resources, num_robots);
    
    % Initialize outputs
    allocation = [];
    collection_paths = struct('robot_id', {}, 'resource_id', {}, 'path_result', {});
    assigned_resources = false(1, num_resources);
    robot_positions = zeros(num_robots, 2);  % Track current positions
    
    % Initialize robot positions
    for i = 1:num_robots
        robot_positions(i, :) = robots(i).position;
    end
    
    % Multi-round allocation
    round_num = 1;
    
    while any(~assigned_resources)
        fprintf('    === Allocation Round %d ===\n', round_num);
        
        % Get available robots and resources
        available_robots = 1:num_robots;  % All robots can take more tasks
        available_resources = find(~assigned_resources);
        
        if isempty(available_resources)
            break;
        end
        
        fprintf('    Available resources: %s\n', sprintf('%d ', available_resources));
        
        % Calculate cost matrix using A* for this round
        cost_matrix = calculate_astar_cost_matrix(robot_positions, resource_list, ...
                                                 available_robots, available_resources, ...
                                                 grid_map, robots, weights);
        
        % Perform auction for this round (assign one resource per available robot)
        round_allocation = [];
        
        % Assign resources in this round
        num_assignments_this_round = min(length(available_robots), length(available_resources));
        
        for assignment = 1:num_assignments_this_round
            % Find minimum cost assignment
            [min_cost, min_idx] = min(cost_matrix(:));
            if isinf(min_cost)
                fprintf('    Warning: No feasible paths found for remaining resources\n');
                break;
            end
            
            [robot_idx, resource_idx] = ind2sub(size(cost_matrix), min_idx);
            actual_robot_id = available_robots(robot_idx);
            actual_resource_id = available_resources(resource_idx);
            
            % Calculate actual path for this assignment
            robot_pos = robot_positions(actual_robot_id, :);
            resource_pos = resource_list(actual_resource_id, :);
            
            try
                path_result = astar_pathfinding(robot_pos, resource_pos, grid_map, ...
                                              robots(actual_robot_id), weights);
                
                % Record allocation
                round_allocation(end+1, :) = [robots(actual_robot_id).id, actual_resource_id, path_result.total_cost];
                
                % Store path with robot and resource mapping
                path_record = struct();
                path_record.robot_id = robots(actual_robot_id).id;
                path_record.resource_id = actual_resource_id;
                path_record.path_result = path_result;
                collection_paths(end+1) = path_record;
                
                % Update robot assignment
                robots(actual_robot_id).assigned_resources(end+1) = actual_resource_id;
                robots(actual_robot_id).current_status = 'assigned';
                
                % Update robot position to the assigned resource
                robot_positions(actual_robot_id, :) = resource_pos;
                robots(actual_robot_id).position = resource_pos;
                
                % Mark resource as assigned
                assigned_resources(actual_resource_id) = true;
                
                fprintf('    Robot %d -> Resource %d (Cost: %.2f, Path: %d waypoints)\n', ...
                        robots(actual_robot_id).id, actual_resource_id, ...
                        path_result.total_cost, path_result.num_waypoints);
                
                % Remove this robot and resource from current round
                cost_matrix(robot_idx, :) = inf;
                cost_matrix(:, resource_idx) = inf;
                
            catch ME
                fprintf('    Warning: Failed to find path for Robot %d to Resource %d: %s\n', ...
                        robots(actual_robot_id).id, actual_resource_id, ME.message);
                cost_matrix(robot_idx, resource_idx) = inf;
                continue;
            end
        end
        
        % Add round results to total allocation
        allocation = [allocation; round_allocation];
        
        fprintf('    Round %d completed: %d assignments made\n', round_num, size(round_allocation, 1));
        round_num = round_num + 1;
        
        % Safety check to prevent infinite loops
        if round_num > num_resources
            fprintf('    Warning: Maximum rounds exceeded, stopping allocation\n');
            break;
        end
    end
    
    % Final summary
    total_assignments = size(allocation, 1);
    total_cost = sum(allocation(:, 3));
    
    fprintf('  === Collection Task Allocation Summary ===\n');
    fprintf('    Total assignments: %d/%d resources\n', total_assignments, num_resources);
    fprintf('    Total cost: %.2f\n', total_cost);
    fprintf('    Average cost per assignment: %.2f\n', total_cost / total_assignments);
    fprintf('    Allocation completed in %d rounds\n', round_num - 1);
    fprintf('    Generated %d path records\n', length(collection_paths));
    
    % Update robot task field for backward compatibility
    for i = 1:num_robots
        if ~isempty(robots(i).assigned_resources)
            robots(i).task = robots(i).assigned_resources(1);  % First assigned resource
        end
    end
    
    updated_robots = robots;
    
end

%% Calculate A* cost matrix for available robots and resources
function cost_matrix = calculate_astar_cost_matrix(robot_positions, resource_list, ...
                                                  available_robots, available_resources, ...
                                                  grid_map, robots, weights)
% Calculate cost matrix using A* path planning
    
    num_available_robots = length(available_robots);
    num_available_resources = length(available_resources);
    cost_matrix = inf(num_available_robots, num_available_resources);
    
    fprintf('    Calculating A* cost matrix (%dx%d)...\n', ...
            num_available_robots, num_available_resources);
    
    for i = 1:num_available_robots
        robot_id = available_robots(i);
        robot_pos = robot_positions(robot_id, :);
        
        for j = 1:num_available_resources
            resource_id = available_resources(j);
            resource_pos = resource_list(resource_id, :);
            
            try
                % Use A* to calculate actual path cost
                path_result = astar_pathfinding(robot_pos, resource_pos, grid_map, ...
                                              robots(robot_id), weights);
                cost_matrix(i, j) = path_result.total_cost;
                
            catch ME
                % If no path found, set cost to infinity
                cost_matrix(i, j) = inf;
                fprintf('      Warning: No path from Robot %d pos [%.1f,%.1f] to Resource %d pos [%.1f,%.1f]\n', ...
                        robots(robot_id).id, robot_pos(1), robot_pos(2), ...
                        resource_id, resource_pos(1), resource_pos(2));
            end
        end
    end
    
    % Display cost matrix for debugging
    fprintf('    Cost matrix calculated (finite costs only):\n');
    for i = 1:num_available_robots
        robot_id = available_robots(i);
        finite_costs = cost_matrix(i, :);
        finite_costs = finite_costs(isfinite(finite_costs));
        if ~isempty(finite_costs)
            fprintf('      Robot %d: min=%.2f, max=%.2f, avg=%.2f\n', ...
                    robots(robot_id).id, min(finite_costs), max(finite_costs), mean(finite_costs));
        end
    end
    
end