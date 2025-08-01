function [updated_robots, transport_results] = execute_transport_mission_silent(robots, transport_paths, base_pos, grid_map)
% Execute transport mission without visualization (silent mode)
% silent version

    % Initialize results
    transport_results = struct();
    transport_results.robot_trajectories = {};
    
    % Process transport paths and group by robot_id
    for i = 1:length(robots)
        robot_id = robots(i).id;
        robot_trajectory = [];
        total_energy_consumed = 0;
        
        % Find all transport tasks for this robot in order
        robot_transport_indices = [];
        for j = 1:length(transport_paths)
            if transport_paths(j).robot_id == robot_id
                robot_transport_indices(end+1) = j;
            end
        end
        
        % Build complete trajectory from all transport tasks
        current_pos = base_pos;  % All robots start from base
        
        for task_idx = robot_transport_indices
            % Get outbound and return paths
            outbound_path = transport_paths(task_idx).outbound_path;
            return_path = transport_paths(task_idx).return_path;
            total_energy_consumed = total_energy_consumed + outbound_path.total_energy + return_path.total_energy;
            
            % Add outbound path (base/current -> resource)
            if isempty(robot_trajectory)
                robot_trajectory = outbound_path.path;
            else
                % Connect to previous trajectory
                robot_trajectory = [robot_trajectory; outbound_path.path(2:end, :)];
            end
            
            % Add return path (resource -> base)
            robot_trajectory = [robot_trajectory; return_path.path(2:end, :)];
            
            % Update current position (should be back at base)
            current_pos = return_path.path(end, :);
        end
        
        % Store trajectory for this robot
        transport_results.robot_trajectories{i} = robot_trajectory;
        robots(i).total_energy = total_energy_consumed;
        robots(i).current_status = 'transport_completed';
    end
    

    % visualize_transport_trajectories(transport_results, robots, base_pos, grid_map);
    
    updated_robots = robots;
end