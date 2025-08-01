function [map, grid_map, color_idx_map, color_idx_map_coarse] = map_generate()
%% === Generate 10km×10km Lunar Map ===
map_size = 10000;
map = ones(map_size);  % Initialize as normal area: 1

%% Set Central Lunar Base
map = addCircleArea(map, 5000, 5000, 100, 8);

%% Set 8 Resource Points
map = addCircleArea(map, 2000, 2000, 50, 9);
map = addCircleArea(map, 8000, 2000, 50, 9);
map = addCircleArea(map, 3000, 5000, 50, 9);
map = addCircleArea(map, 7000, 6000, 50, 9);
map = addCircleArea(map, 1000, 9000, 50, 9);
map = addCircleArea(map, 9000, 9000, 50, 9);
map = addCircleArea(map, 2500, 7500, 50, 9);
map = addCircleArea(map, 7500, 1500, 50, 9);

%% Set Different Energy-consumption Areas
% 3x energy consumption areas
map = addCircleArea(map, 4800, 2600, 1400, 3);
map = addRectArea(map, 7800, 4900, 4600, 2000, 3);

% 4x energy consumption areas
map = addCircleArea(map, 5700, 7700, 2000, 4);

%% Set Obstacles
map = addRectArea(map, 1500, 6300, 400, 6000, 0);
map = addRectArea(map, 1950, 7500, 500, 3000, 0);
map = addRectArea(map, 3700, 6500, 900, 600, 0);
map = addCircleArea(map, 7800, 7600, 800, 0);
map = addRectArea(map, 8000, 2500, 5000, 200, 0);
map = addRectArea(map, 8900, 2600, 200, 2800, 0);

%% Generate Color Index Map (for visualization)
unique_vals = [0, 1, 3, 4, 8, 9];
color_idx_map = zeros(size(map));
for k = 1:length(unique_vals)
    color_idx_map(map == unique_vals(k)) = k;
end

%% Generate 200×200 Coarse Grid Map (for A* algorithm)
block_size = 50; % Each grid cell: 50m×50m
grid_map = generateGridMap(map, block_size);

% Generate coarse grid color index
color_idx_map_coarse = zeros(size(grid_map));
for k = 1:length(unique_vals)
    color_idx_map_coarse(grid_map == unique_vals(k)) = k;
end

fprintf('Map generation completed: %dx%d original map, %dx%d grid map\n', ...
        size(map,1), size(map,2), size(grid_map,1), size(grid_map,2));

end

%% === Grid Map Generation Function ===
function grid_map = generateGridMap(map, block_size)
    [H, W] = size(map);
    grid_H = H / block_size;
    grid_W = W / block_size;
    grid_map = zeros(grid_H, grid_W);

    for i = 1:grid_H
        for j = 1:grid_W
            block = map((i-1)*block_size+1:i*block_size, ...
                        (j-1)*block_size+1:j*block_size);

            % Priority: obstacles>base>resource points>4x energy>3x energy>normal
            if any(block(:) == 0)
                grid_map(i,j) = 0;
            elseif any(block(:) == 8)
                grid_map(i,j) = 8;
            elseif any(block(:) == 9)
                grid_map(i,j) = 9;
            elseif any(block(:) == 4)
                grid_map(i,j) = 4;
            elseif any(block(:) == 3)
                grid_map(i,j) = 3;
            else
                grid_map(i,j) = 1;
            end
        end
    end
end

%% === Add Rectangular Area ===
function map = addRectArea(map, centerX, centerY, width, height, value)
    x1 = max(1, round(centerX - width / 2));
    x2 = min(size(map,2), round(centerX + width / 2));
    y1 = max(1, round(centerY - height / 2));
    y2 = min(size(map,1), round(centerY + height / 2));
    
    map(y1:y2, x1:x2) = value;
end

%% === Add Circular Area ===
function map = addCircleArea(map, centerX, centerY, radius, value)
    [H, W] = size(map);
    [X, Y] = meshgrid(1:W, 1:H);
    mask = (X - centerX).^2 + (Y - centerY).^2 <= radius^2;
    map(mask) = value;
end