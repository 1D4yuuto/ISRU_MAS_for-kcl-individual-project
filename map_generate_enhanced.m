function [map, grid_map, color_idx_map, color_idx_map_coarse] = map_generate_enhanced()
%% === Generate Enhanced 10km×10km Lunar Map with Strategic Terrain ===
map_size = 10000;
map = ones(map_size);  % Initialize as normal area: 1

%% Set Central Lunar Base (保持相同位置)
map = addCircleArea(map, 5000, 5000, 100, 8);

%% Set 8 Resource Points (保持相同位置)
map = addCircleArea(map, 2000, 2000, 50, 9);
map = addCircleArea(map, 8000, 2000, 50, 9);
map = addCircleArea(map, 3000, 5000, 50, 9);
map = addCircleArea(map, 7000, 6000, 50, 9);
map = addCircleArea(map, 1000, 9000, 50, 9);
map = addCircleArea(map, 9000, 9000, 50, 9);
map = addCircleArea(map, 2500, 7500, 50, 9);
map = addCircleArea(map, 7500, 1500, 50, 9);

%% Enhanced Strategic Energy-consumption Areas
% 原有区域保持 - 但调整大小
map = addCircleArea(map, 4800, 2600, 1200, 3);    % 3x energy (稍小)
map = addRectArea(map, 7800, 4900, 4000, 1800, 3); % 3x energy (稍小)
map = addCircleArea(map, 5700, 7700, 1800, 4);     % 4x energy (稍小)

% *** 新增战略性高能耗区域 ***

% 1. 基地周围的"十字形"高能耗带
map = addRectArea(map, 4000, 5000, 1800, 500, 4);  % 基地西侧4x带
map = addRectArea(map, 6200, 5000, 1800, 500, 4);  % 基地东侧4x带
map = addRectArea(map, 5000, 3800, 500, 1700, 3);  % 基地南侧3x带
map = addRectArea(map, 5000, 6500, 500, 1700, 3);  % 基地北侧3x带

% 2. 阻挡关键路径的战略区域
map = addCircleArea(map, 3500, 3500, 700, 4);      % 阻挡基地到R2的直路
map = addRectArea(map, 6000, 3000, 800, 1000, 4);  % 阻挡基地到R4的直路
map = addCircleArea(map, 2200, 6800, 600, 3);      % 阻挡基地到R3的路径
map = addRectArea(map, 7200, 7000, 600, 800, 3);   % 阻挡基地到R5的路径

% 3. 创建"能耗走廊"迫使路径选择
map = addRectArea(map, 3000, 6500, 1000, 300, 4);  % 北部4x走廊
map = addRectArea(map, 6500, 3500, 300, 1000, 4);  % 东部4x走廊
map = addRectArea(map, 2000, 4000, 1200, 300, 3);  % 西南3x走廊
map = addRectArea(map, 7000, 7500, 1000, 300, 3);  % 东北3x走廊

% 4. 边缘高能耗区域减少绕行空间
map = addRectArea(map, 800, 5000, 400, 6000, 3);   % 西边界3x带
map = addRectArea(map, 9200, 5000, 400, 6000, 3);  % 东边界3x带
map = addRectArea(map, 5000, 800, 6000, 400, 3);   % 南边界3x带
map = addRectArea(map, 5000, 9200, 6000, 400, 3);  % 北边界3x带

%% Set Same Obstacles (保持原有障碍物)
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
grid_map = generateGridMap(map, block_size, 0.2, 0.4); % 使用相同阈值

% Generate coarse grid color index
color_idx_map_coarse = generate_color_map(grid_map);

fprintf('Enhanced map generation completed: %dx%d original map, %dx%d grid map\n', ...
        size(map,1), size(map,2), size(grid_map,1), size(grid_map,2));

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