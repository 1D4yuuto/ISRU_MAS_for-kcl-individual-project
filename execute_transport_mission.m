function [updated_robots, transport_results] = execute_transport_mission(robots, transport_paths, base_pos, grid_map)
% Execute transport mission with static trajectory visualization
% Input:
%   robots          - array of transporter robots with assigned tasks
%   transport_paths - struct array with outbound and return paths
%   base_pos        - base position [x, y]
%   grid_map        - 200x200 grid map
% Output:
%   updated_robots    - robots with updated statistics
%   transport_results - mission execution results

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
    
    % Generate visualization
    visualize_transport_trajectories(transport_results, robots, base_pos, grid_map);
    
    updated_robots = robots;
end

%% Visualize transport trajectories
function visualize_transport_trajectories(transport_results, robots, base_pos, grid_map)
% Create static visualization of all transport trajectories
    
    % Use same method as Step 4 for consistent visualization
    cmap = [
        0 0 0;        % 0: Black - Obstacles
        1 1 1;        % 1: White - Normal Area
        0.8 0.6 0.2;  % 3: Grey - 3x Energy-consumption
        0.6 0.2 0.2;  % 4: Dark Grey - 4x Energy-consumption
        1 1 0;        % 8: Yellow - Lunar Base
        0 1 1         % 9: Cyan - Resource Points
    ];
    
    % Generate color index map
    unique_vals = [0, 1, 3, 4, 8, 9];
    color_idx_map_coarse = zeros(size(grid_map));
    for k = 1:length(unique_vals)
        color_idx_map_coarse(grid_map == unique_vals(k)) = k;
    end
    
    % Get clustered resource list for marking
    [res_y, res_x] = find(grid_map == 9);
    raw_resources = [res_x, res_y];
    resource_list = cluster_resources(raw_resources);
    
    % Create figure
    figure('Name', 'Transport Mission Trajectories', 'Position', [950, 100, 800, 700]);
    imagesc(color_idx_map_coarse);
    colormap(cmap);
    axis equal tight;
    set(gca, 'YDir', 'normal');
    hold on;
    
    % Mark base
    plot(base_pos(1), base_pos(2), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    text(base_pos(1)+5, base_pos(2)+5, 'BASE', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'red');
    
    % Mark clustered resources (numbered 1-8)
    for i = 1:size(resource_list, 1)
        plot(resource_list(i,1), resource_list(i,2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', 'LineWidth', 2);
        text(resource_list(i,1)+3, resource_list(i,2)+3, sprintf('R%d', i), 'FontSize', 11, 'Color', 'blue', 'FontWeight', 'bold');
    end
    
    % 改进的运输机器人颜色（使用更加区分明显的颜色，参考收集机器人的配色方案）
    robot_colors = [
        1.0, 0.0, 0.0;  % 红色 for Robot 6 (Transport 1)
        0.0, 0.0, 1.0;  % 蓝色 for Robot 7 (Transport 2)  
        1.0, 0.5, 0.0;  % 橙色 for Robot 8 (Transport 3)
    ];
    
    % Plot all transport robot trajectories
    for robot_idx = 1:length(robots)
        trajectory = transport_results.robot_trajectories{robot_idx};
        
        if ~isempty(trajectory) && size(trajectory, 1) > 1
            color_idx = mod(robot_idx-1, size(robot_colors, 1)) + 1;
            
            % 使用实线而不是虚线，线条更粗以便区分
            plot(trajectory(:,1), trajectory(:,2), '-', ...
                 'Color', robot_colors(color_idx,:), 'LineWidth', 4);
            
            % Mark start point (diamond shape for transport robots)
            plot(trajectory(1,1), trajectory(1,2), 'd', ...
                 'Color', robot_colors(color_idx,:), 'MarkerSize', 12, ...
                 'MarkerFaceColor', robot_colors(color_idx,:), 'LineWidth', 2);
            
            % Mark end point (hexagram for transport robots)  
            plot(trajectory(end,1), trajectory(end,2), 'h', ...
                 'Color', robot_colors(color_idx,:), 'MarkerSize', 12, ...
                 'MarkerFaceColor', robot_colors(color_idx,:), 'LineWidth', 2);
            
            % Add Robot label (T for Transport) 使用更明显的标签样式
            text(trajectory(1,1)+4, trajectory(1,2)+4, sprintf('T%d', robots(robot_idx).id), ...
                 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'white', ...
                 'BackgroundColor', robot_colors(color_idx,:), 'EdgeColor', 'black', ...
                 'Margin', 2);
        end
    end
    
    title('Transport Mission - Robot Trajectories', 'FontSize', 16, 'FontWeight', 'bold');
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
    
    % Add legend for transport robots
    legend_handles = [];
    legend_labels = {};
    for robot_idx = 1:min(length(robots), 3)  % Show up to 3 robots in legend
        color_idx = mod(robot_idx-1, size(robot_colors, 1)) + 1;
        h = plot(NaN, NaN, '-', 'Color', robot_colors(color_idx,:), 'LineWidth', 4);
        legend_handles(end+1) = h;
        legend_labels{end+1} = sprintf('Transport Robot %d', robots(robot_idx).id);
    end
    
    if ~isempty(legend_handles)
        legend(legend_handles, legend_labels, 'Location', 'northeast', 'FontSize', 10);
    end
    
    hold off;
    
end