function grid_map = generateGridmap_silent(map, block_size, obstacle_threshold, energy_threshold)
% Generate improved grid map with coverage-based rules (silent mode)
% silent version
% Input:
%   map - 10000x10000 original map
%   block_size - size of each grid cell (50 meters)
%   obstacle_threshold - obstacle coverage threshold (0.2 = 20%)
%   energy_threshold - energy terrain coverage threshold (0.4 = 40%)
% Output:
%   grid_map - 200x200 grid map for A* algorithm

    [H, W] = size(map);
    grid_H = H / block_size;
    grid_W = W / block_size;
    grid_map = zeros(grid_H, grid_W);
    
    % fprintf('  Applying coverage rules: %.0f%% obstacle, %.0f%% energy terrain\n', ...
    %         obstacle_threshold * 100, energy_threshold * 100);

    for i = 1:grid_H
        for j = 1:grid_W
            % Extract current block (50x50 pixels)
            block = map((i-1)*block_size+1:i*block_size, ...
                        (j-1)*block_size+1:j*block_size);
            
            % Calculate coverage ratios for different terrain types
            total_pixels = numel(block);
            obstacle_ratio = sum(block(:) == 0) / total_pixels;
            energy_3_ratio = sum(block(:) == 3) / total_pixels;
            energy_4_ratio = sum(block(:) == 4) / total_pixels;
            base_ratio = sum(block(:) == 8) / total_pixels;
            resource_ratio = sum(block(:) == 9) / total_pixels;
            
            % Apply priority-based classification with coverage rules
            if obstacle_ratio > obstacle_threshold
                % If obstacle coverage > 20%, mark as obstacle
                grid_map(i,j) = 0;
            elseif base_ratio > 0
                % Any base presence -> base grid (highest priority after obstacles)
                grid_map(i,j) = 8;
            elseif resource_ratio > 0
                % Any resource presence -> resource grid
                grid_map(i,j) = 9;
            elseif energy_4_ratio > energy_threshold
                % If 4x energy terrain coverage > 40%, mark as 4x energy
                grid_map(i,j) = 4;
            elseif energy_3_ratio > energy_threshold
                % If 3x energy terrain coverage > 40%, mark as 3x energy
                grid_map(i,j) = 3;
            else
                % Default: normal terrain
                grid_map(i,j) = 1;
            end
        end
    end
    
    % total_grids = grid_H * grid_W;
    % obstacle_count = sum(grid_map(:) == 0);
    % normal_count = sum(grid_map(:) == 1);
    % energy_3_count = sum(grid_map(:) == 3);
    % energy_4_count = sum(grid_map(:) == 4);
    % base_count = sum(grid_map(:) == 8);
    % resource_count = sum(grid_map(:) == 9);
    % 
    % fprintf('  Grid map statistics:\n');
    % fprintf('    Obstacles: %d grids (%.1f%%)\n', obstacle_count, obstacle_count/total_grids*100);
    % fprintf('    Normal areas: %d grids (%.1f%%)\n', normal_count, normal_count/total_grids*100);
    % fprintf('    3x energy areas: %d grids (%.1f%%)\n', energy_3_count, energy_3_count/total_grids*100);
    % fprintf('    4x energy areas: %d grids (%.1f%%)\n', energy_4_count, energy_4_count/total_grids*100);
    % fprintf('    Base grids: %d\n', base_count);
    % fprintf('    Resource grids: %d\n', resource_count);
    
end