function [updated_robots, collection_results] = execute_collection_mission(robots, collection_paths, base_pos, grid_map)
% Execute collection mission with static trajectory visualization

    % Initialize results
    collection_results = struct();
    collection_results.robot_trajectories = {};
    
    % Now collection_paths is a struct array with robot_id, resource_id, and path_result
    % Group paths by robot_id for correct trajectory building
    for i = 1:length(robots)
        robot_id = robots(i).id;
        robot_trajectory = [];
        total_energy_consumed = 0;
        
        % Find all paths for this robot in the correct order
        robot_paths = [];
        for j = 1:length(collection_paths)
            if collection_paths(j).robot_id == robot_id
                robot_paths(end+1) = j;  % Store index of path for this robot
            end
        end
        
        % Build trajectory from robot's paths in order
        for path_idx = robot_paths
            path_result = collection_paths(path_idx).path_result;  % Get the path_result struct
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
    
    % Generate visualization
    visualize_trajectories_simple(collection_results, robots, base_pos, grid_map);
    
    updated_robots = robots;
end

%% Simple trajectory visualization
function visualize_trajectories_simple(collection_results, robots, base_pos, grid_map)
    
    % Use same method as display_maps for consistent visualization
    cmap = [
        0 0 0;        % 0: Black - Obstacles
        1 1 1;        % 1: White - Normal Area
        0.8 0.6 0.2;  % 3: Grey - 3x Energy-consumption
        0.6 0.2 0.2;  % 4: Dark Grey - 4x Energy-consumption
        1 1 0;        % 8: Yellow - Lunar Base
        0 1 1         % 9: Cyan - Resource Points
    ];
    
    % Generate color index map (same as display_maps)
    unique_vals = [0, 1, 3, 4, 8, 9];
    color_idx_map_coarse = zeros(size(grid_map));
    for k = 1:length(unique_vals)
        color_idx_map_coarse(grid_map == unique_vals(k)) = k;
    end
    
    % Get clustered resource list (fix the clustering issue)
    [res_y, res_x] = find(grid_map == 9);
    raw_resources = [res_x, res_y];
    resource_list = cluster_resources(raw_resources);  % Apply clustering
    
    %% Figure: Grid Map with Trajectories
    figure('Name', 'Collection Mission Trajectories', 'Position', [950, 100, 800, 700]);
    imagesc(color_idx_map_coarse);
    colormap(cmap);
    axis equal tight;
    set(gca, 'YDir', 'normal');
    hold on;
    
    % Mark base on grid map
    plot(base_pos(1), base_pos(2), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    text(base_pos(1)+5, base_pos(2)+5, 'BASE', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'red');
    
    % Mark clustered resources (numbered 1-8)
    for i = 1:size(resource_list, 1)
        plot(resource_list(i,1), resource_list(i,2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', 'LineWidth', 2);
        text(resource_list(i,1)+3, resource_list(i,2)+3, sprintf('R%d', i), 'FontSize', 11, 'Color', 'blue', 'FontWeight', 'bold');
    end
    
    % Robot colors for all 5 robots（保持原有颜色方案）
    robot_colors = [
        0.0, 0.8, 0.0;  % Green for Robot 1
        1.0, 0.5, 0.0;  % Orange for Robot 2
        0.8, 0.0, 0.8;  % Magenta for Robot 3
        0.0, 0.5, 1.0;  % Blue for Robot 4
        1.0, 0.8, 0.0;  % Yellow for Robot 5
    ];
    
    % Plot all 5 robot trajectories
    for robot_idx = 1:5  % Show all 5 robots
        trajectory = collection_results.robot_trajectories{robot_idx};
        
        if ~isempty(trajectory) && size(trajectory, 1) > 1
            % Plot trajectory (线条稍微加粗)
            plot(trajectory(:,1), trajectory(:,2), '-', ...
                 'Color', robot_colors(robot_idx,:), 'LineWidth', 4);
            
            % Mark start point (circle)
            plot(trajectory(1,1), trajectory(1,2), 'o', ...
                 'Color', robot_colors(robot_idx,:), 'MarkerSize', 10, ...
                 'MarkerFaceColor', robot_colors(robot_idx,:), 'LineWidth', 2);
            
            % Mark end point (square)
            plot(trajectory(end,1), trajectory(end,2), 's', ...
                 'Color', robot_colors(robot_idx,:), 'MarkerSize', 10, ...
                 'MarkerFaceColor', robot_colors(robot_idx,:), 'LineWidth', 2);
            
            % Add Robot label（改进标签样式，和运输机器人保持一致）
            text(trajectory(1,1)+4, trajectory(1,2)+4, sprintf('C%d', robot_idx), ...
                 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'white', ...
                 'BackgroundColor', robot_colors(robot_idx,:), 'EdgeColor', 'black', ...
                 'Margin', 2);
        end
    end
    
    title('Collection Mission - All Robot Trajectories', 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('X-grid index', 'FontSize', 14);
    ylabel('Y-grid index', 'FontSize', 14);
    
    % Add only major grid lines (every 20 cells for better visibility)
    xticks(0:20:200);
    yticks(0:20:200);
    grid on;
    set(gca, 'GridColor', [0.7 0.7 0.7], 'GridAlpha', 0.3);
    
    % Add colorbar
    colorbar('Ticks', 1:6, ...
             'TickLabels', {'Obstacles', 'Normal', '3x Energy', '4x Energy', 'Base', 'Resources'}, ...
             'FontSize', 12);
    
    % 添加收集机器人图例（和运输机器人风格一致）
    legend_handles = [];
    legend_labels = {};
    for robot_idx = 1:5  % Show all 5 collection robots in legend
        h = plot(NaN, NaN, '-', 'Color', robot_colors(robot_idx,:), 'LineWidth', 4);
        legend_handles(end+1) = h;
        legend_labels{end+1} = sprintf('Collection Robot %d', robot_idx);
    end
    
    if ~isempty(legend_handles)
        legend(legend_handles, legend_labels, 'Location', 'northeast', 'FontSize', 10);
    end
    
    hold off;
    
end