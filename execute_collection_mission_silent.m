function [updated_robots, collection_results] = execute_collection_mission_silent(robots, collection_paths, base_pos, grid_map)
% Execute collection mission without visualization (silent mode)
% silent version

    % Initialize results
    collection_results = struct();
    collection_results.robot_trajectories = {};
    
    % Process collection paths and build trajectories
    for i = 1:length(robots)
        robot_id = robots(i).id;
        robot_trajectory = [];
        total_energy_consumed = 0;
        
        % Find all paths for this robot in the correct order
        robot_paths = [];
        for j = 1:length(collection_paths)
            if collection_paths(j).robot_id == robot_id
                robot_paths(end+1) = j;
            end
        end
        
        % Build trajectory from robot's paths in order
        for path_idx = robot_paths
            path_result = collection_paths(path_idx).path_result;
            total_energy_consumed = total_energy_consumed + path_result.total_energy;
            
            if isempty(robot_trajectory)
                robot_trajectory = path_result.path;
            else
                % Connect paths
                robot_trajectory = [robot_trajectory; path_result.path(2:end, :)];
            end
        end
        
        % Add return path to base
        if ~isempty(robot_trajectory)
            last_pos = robot_trajectory(end, :);
            if ~isequal(last_pos, base_pos)
                try
                    return_weights = struct('distance', 1.0, 'time', 0.0, 'energy', 0.0);
                    return_path = astar_pathfinding(last_pos, base_pos, grid_map, robots(i), return_weights);
                    robot_trajectory = [robot_trajectory; return_path.path(2:end, :)];
                catch
                    robot_trajectory = [robot_trajectory; base_pos];
                end
            end
        end
        
        collection_results.robot_trajectories{i} = robot_trajectory;
        robots(i).total_energy = total_energy_consumed;
        robots(i).current_status = 'completed';
    end
    

    % visualize_trajectories_simple(collection_results, robots, base_pos, grid_map);
    
    updated_robots = robots;
end