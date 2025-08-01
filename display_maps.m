function display_maps(map, color_idx_map, grid_map, color_idx_map_coarse, base_pos, resource_list)
% Display original map and grid map in separate figures for better visualization
% Input:
%   map - 10000x10000 original map
%   color_idx_map - color index for original map
%   grid_map - 200x200 grid map for A*
%   color_idx_map_coarse - color index for grid map
%   base_pos - base position [x, y]
%   resource_list - resource positions [x, y; ...]

    % Define color mapping
    cmap = [
        0 0 0;        % 0: Black - Obstacles
        1 1 1;        % 1: White - Normal Area
        0.8 0.6 0.2;  % 3: Grey - 3x Energy-consumption
        0.6 0.2 0.2;  % 4: Dark Grey - 4x Energy-consumption
        1 1 0;        % 8: Yellow - Lunar Base
        0 1 1         % 9: Cyan - Resource Points
    ];
    
    %% Figure 1: Original Map (10000x10000)
    figure('Name', 'Original Lunar Map', 'Position', [100, 100, 800, 700]);
    imagesc(color_idx_map);
    colormap(cmap);
    axis equal tight;
    set(gca, 'YDir', 'normal');
    hold on;
    
    % Mark base on original map
    plot(base_pos(1)*50, base_pos(2)*50, 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    text(base_pos(1)*50+200, base_pos(2)*50+200, 'BASE', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'red');
    
    % Mark resources on original map
    for i = 1:size(resource_list, 1)
        plot(resource_list(i,1)*50, resource_list(i,2)*50, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'blue');
        text(resource_list(i,1)*50+150, resource_list(i,2)*50+150, sprintf('R%d', i), 'FontSize', 10, 'Color', 'blue', 'FontWeight', 'bold');
    end
    
    title('Original Lunar Map (10km × 10km)', 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('X-coordinate (m)', 'FontSize', 14);
    ylabel('Y-coordinate (m)', 'FontSize', 14);
    
    % Add major grid for better visualization (every 1km)
    xticks(0:1000:10000);
    yticks(0:1000:10000);
    grid on;
    set(gca, 'GridColor', [0.6 0.6 0.6], 'GridAlpha', 0.4, 'LineWidth', 0.5);
    
    % Add colorbar
    colorbar('Ticks', 1:6, ...
             'TickLabels', {'Obstacles', 'Normal', '3x Energy', '4x Energy', 'Base', 'Resources'}, ...
             'FontSize', 12);
    
    hold off;

    %% Figure 2: Grid Map (200x200) with Full Grid Display
    figure('Name', 'Grid Map for A* Algorithm', 'Position', [950, 100, 800, 700]);
    imagesc(color_idx_map_coarse);
    colormap(cmap);
    axis equal tight;
    set(gca, 'YDir', 'normal');
    hold on;
    
    % Mark base on grid map
    plot(base_pos(1), base_pos(2), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 3);
    text(base_pos(1)+5, base_pos(2)+5, 'BASE', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'red');
    
    % Mark resources on grid map
    for i = 1:size(resource_list, 1)
        plot(resource_list(i,1), resource_list(i,2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'blue', 'LineWidth', 2);
        text(resource_list(i,1)+3, resource_list(i,2)+3, sprintf('R%d', i), 'FontSize', 11, 'Color', 'blue', 'FontWeight', 'bold');
    end
    
    title('Grid Map for A* Algorithm (200 × 200, 50m/grid)', 'FontSize', 16, 'FontWeight', 'bold');
    xlabel('X-grid index', 'FontSize', 14);
    ylabel('Y-grid index', 'FontSize', 14);
    
    % Add grid to show actual 200x200 cells
    xticks(0.5:10:200.5);  % Show major grid lines every 10 cells for clarity
    yticks(0.5:10:200.5);
    grid on;
    set(gca, 'GridColor', [0.5 0.5 0.5], 'GridAlpha', 0.8, 'LineWidth', 0.8);
    
    % Add minor grid to show individual cells (every cell)
    % Vertical lines for individual cells
    for x = 0.5:200.5
        if mod(x-0.5, 10) ~= 0  % Skip major grid lines
            line([x x], [0.5 200.5], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.1);
        end
    end
    % Horizontal lines for individual cells  
    for y = 0.5:200.5
        if mod(y-0.5, 10) ~= 0  % Skip major grid lines
            line([0.5 200.5], [y y], 'Color', [0.8 0.8 0.8], 'LineWidth', 0.1);
        end
    end
    
    % Add colorbar
    colorbar('Ticks', 1:6, ...
             'TickLabels', {'Obstacles', 'Normal', '3x Energy', '4x Energy', 'Base', 'Resources'}, ...
             'FontSize', 12);
    
    hold off;
    
    %% Print map statistics
    fprintf('  === Map Display Statistics ===\n');
    fprintf('  Original map: %dx%d pixels (%.1f km × %.1f km)\n', ...
            size(map,1), size(map,2), size(map,1)/1000, size(map,2)/1000);
    fprintf('  Grid map: %dx%d grids (%.0fm per grid)\n', ...
            size(grid_map,1), size(grid_map,2), 50);
    fprintf('  Total coverage: %.1f km²\n', (size(grid_map,1)*50/1000) * (size(grid_map,2)*50/1000));
    fprintf('  Terrain distribution:\n');
    fprintf('    Obstacles: %d grids (%.1f%%)\n', ...
            sum(grid_map(:) == 0), sum(grid_map(:) == 0)/numel(grid_map)*100);
    fprintf('    Normal areas: %d grids (%.1f%%)\n', ...
            sum(grid_map(:) == 1), sum(grid_map(:) == 1)/numel(grid_map)*100);
    fprintf('    3x energy areas: %d grids (%.1f%%)\n', ...
            sum(grid_map(:) == 3), sum(grid_map(:) == 3)/numel(grid_map)*100);
    fprintf('    4x energy areas: %d grids (%.1f%%)\n', ...
            sum(grid_map(:) == 4), sum(grid_map(:) == 4)/numel(grid_map)*100);
    fprintf('  Special locations:\n');
    fprintf('    Base: %d grid\n', sum(grid_map(:) == 8));
    fprintf('    Resource points: %d grids\n', sum(grid_map(:) == 9));
    
end