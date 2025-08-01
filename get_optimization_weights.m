function weights = get_optimization_weights(mode)
% Get optimization weights for different optimization strategies
% Input:
%   mode - optimization mode: 'distance', 'time', 'energy', 'hybrid'
% Output:
%   weights - struct containing weight parameters for cost calculation
%
% Weight Structure:
%   .distance - weight for distance cost
%   .time     - weight for time cost  
%   .energy   - weight for energy cost

    switch lower(mode)
        case 'distance'
            % Distance optimization: minimize total travel distance
            weights = struct(...
                'distance', 1.0, ...
                'time', 0.0, ...
                'energy', 0.0);
            
        case 'time'
            % Time optimization: minimize total mission time
            weights = struct(...
                'distance', 0.0, ...
                'time', 1.0, ...
                'energy', 0.0);
            
        case 'energy'
            % Energy optimization: minimize total energy consumption
            weights = struct(...
                'distance', 0.0, ...
                'time', 0.0, ...
                'energy', 1.0);
            
        case 'hybrid'
            % Hybrid optimization: balanced consideration of all factors
            weights = struct(...
                'distance', 0.3, ...
                'time', 0.4, ...
                'energy', 0.3);
            
        otherwise
            % Default to distance optimization with warning
            fprintf('  Warning: Unknown optimization mode "%s", using distance mode\n', mode);
            weights = struct(...
                'distance', 1.0, ...
                'time', 0.0, ...
                'energy', 0.0);
    end
    
    % Print selected optimization strategy
    fprintf('  Optimization weights configured:\n');
    fprintf('    Distance weight: %.1f\n', weights.distance);
    fprintf('    Time weight: %.1f\n', weights.time);
    fprintf('    Energy weight: %.1f\n', weights.energy);
    fprintf('    Strategy: %s optimization\n', upper(mode));
    
end