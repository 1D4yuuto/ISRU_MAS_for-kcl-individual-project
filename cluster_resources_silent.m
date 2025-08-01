function clustered_resources = cluster_resources_silent(raw_resources)
% Cluster adjacent resource grids into single resource points (silent mode)
% silent version
% Input: raw_resources - [x, y] original resource grid positions
% Output: clustered_resources - [x, y] central points after clustering

    if isempty(raw_resources)
        clustered_resources = [];
        return;
    end
    
    % Simple distance-based clustering
    min_distance = 3; % Regard as single resource if within 3 grids in 200x200 map
    clustered_resources = [];
    used = false(size(raw_resources, 1), 1);
    
    for i = 1:size(raw_resources, 1)
        if used(i)
            continue;
        end
        
        % Find all nearby points within clustering distance
        current_point = raw_resources(i, :);
        distances = sqrt(sum((raw_resources - current_point).^2, 2));
        cluster_indices = find(distances <= min_distance & ~used);
        
        % Calculate cluster center
        cluster_points = raw_resources(cluster_indices, :);
        cluster_center = mean(cluster_points, 1);
        
        % Add to results
        clustered_resources(end+1, :) = round(cluster_center);
        
        % Mark these points as used
        used(cluster_indices) = true;
    end
    

    % fprintf('  Clustered %d raw grids into %d resource points\n', ...
    %         size(raw_resources,1), size(clustered_resources,1));
end