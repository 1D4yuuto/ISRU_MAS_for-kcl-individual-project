function color_idx_map = generate_color_map(grid_map)
% Generate color index map for visualization
% Input:
%   grid_map - 200x200 grid map with terrain values
% Output:
%   color_idx_map - color index map for visualization
%
% Terrain Value Mapping:
%   0 -> 1 (Black: Obstacles)
%   1 -> 2 (White: Normal Area)
%   3 -> 3 (Grey: 3x Energy-consumption)
%   4 -> 4 (Dark Grey: 4x Energy-consumption)
%   8 -> 5 (Yellow: Lunar Base)
%   9 -> 6 (Cyan: Resource Points)

    % Define the mapping from terrain values to color indices
    unique_vals = [0, 1, 3, 4, 8, 9];
    color_indices = [1, 2, 3, 4, 5, 6];
    
    % Initialize color index map
    color_idx_map = zeros(size(grid_map));
    
    % Map each terrain value to its corresponding color index
    for k = 1:length(unique_vals)
        terrain_value = unique_vals(k);
        color_index = color_indices(k);
        color_idx_map(grid_map == terrain_value) = color_index;
    end
    
    % Print mapping statistics
    fprintf('  Color mapping generated:\n');
    for k = 1:length(unique_vals)
        terrain_value = unique_vals(k);
        color_index = color_indices(k);
        count = sum(grid_map(:) == terrain_value);
        
        % Define terrain type names for display
        switch terrain_value
            case 0
                terrain_name = 'Obstacles';
            case 1
                terrain_name = 'Normal areas';
            case 3
                terrain_name = '3x energy areas';
            case 4
                terrain_name = '4x energy areas';
            case 8
                terrain_name = 'Base';
            case 9
                terrain_name = 'Resource points';
            otherwise
                terrain_name = 'Unknown';
        end
        
        if count > 0
            fprintf('    %s: %d grids -> color index %d\n', ...
                    terrain_name, count, color_index);
        end
    end
    
end