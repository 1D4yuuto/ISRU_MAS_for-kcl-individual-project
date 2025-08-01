function [collector_robots, transporter_robots] = init_robots(base_pos, num_collectors, num_transporters)
% Initialize collector robots and transporter robots with enhanced tracking
% Input:
%   base_pos          : robot initial position [x, y] in grid coordinates
%   num_collectors    : number of collector robots
%   num_transporters  : number of transporter robots
% Output:
%   collector_robots  : collector robot struct array
%   transporter_robots: transporter robot struct array

    %% Initialize Collector Robots
    collector_robots(num_collectors) = struct();
    for i = 1:num_collectors
        %% Basic robot properties
        collector_robots(i).id = i;
        collector_robots(i).type = 'collector';
        collector_robots(i).position = base_pos;        % Initial position = base
        collector_robots(i).speed = 0.12;               % m/s
        collector_robots(i).energy_move = 5;            % W/s
        collector_robots(i).energy_sense = 2;           % W/s
        collector_robots(i).range_sense = 10;           % Sense range/m
        collector_robots(i).task = [];                  % No initial task
        collector_robots(i).status = 'idle';            % Initial state
        
        %% Enhanced performance tracking fields
        collector_robots(i).total_time = 0;        % Total mission time (seconds)
        collector_robots(i).total_distance = 0;    % Total travel distance (meters)
        collector_robots(i).total_energy = 0;      % Total energy consumption (Joules)
        
        %% Path and status tracking
        collector_robots(i).path_history = base_pos;           % Path coordinate history [x,y; x,y; ...]
        collector_robots(i).current_status = 'idle';           % Current status: 'idle', 'moving', 'collecting'
        collector_robots(i).assigned_resources = [];           % List of assigned resource IDs
        collector_robots(i).completed_resources = [];          % List of completed resource IDs
        
        %% Mission timing
        collector_robots(i).mission_start_time = 0;           % Mission start timestamp
        collector_robots(i).last_update_time = 0;             % Last status update timestamp
        
        %% A* path planning results
        collector_robots(i).current_path = [];                % Current planned path
        collector_robots(i).path_costs = struct(...           % Path cost breakdown
            'distance_cost', 0, ...
            'time_cost', 0, ...
            'energy_cost', 0, ...
            'total_cost', 0);
    end

    %% Initialize Transporter Robots
    transporter_robots(num_transporters) = struct();
    for i = 1:num_transporters
        %% Basic robot properties
        transporter_robots(i).id = num_collectors + i;
        transporter_robots(i).type = 'transporter';
        transporter_robots(i).position = base_pos;
        transporter_robots(i).speed = 0.2;              % m/s
        transporter_robots(i).energy_move = 6;          % W/s
        transporter_robots(i).energy_sense = 1;         % W/s
        transporter_robots(i).range_sense = 5;          % Sense range/m
        transporter_robots(i).task = [];                % No initial task
        transporter_robots(i).status = 'idle';          % Initial state
        
        %% Enhanced performance tracking fields
        transporter_robots(i).total_time = 0;        % Total mission time (seconds)
        transporter_robots(i).total_distance = 0;    % Total travel distance (meters)
        transporter_robots(i).total_energy = 0;      % Total energy consumption (Joules)
        
        %% Path and status tracking
        transporter_robots(i).path_history = base_pos;         % Path coordinate history
        transporter_robots(i).current_status = 'idle';         % Current status: 'idle', 'moving', 'transporting'
        transporter_robots(i).assigned_resources = [];         % List of assigned resource IDs for transport
        transporter_robots(i).completed_transports = [];       % List of completed transport IDs
        
        %% Transport-specific fields
        transporter_robots(i).transport_queue = [];            % Queue of transport tasks
        transporter_robots(i).round_trips = 0;                 % Number of completed round trips
        transporter_robots(i).current_destination = [];        % Current destination [x, y]
        transporter_robots(i).is_returning = false;            % Flag: returning to base or going to resource
        
        %% Mission timing
        transporter_robots(i).mission_start_time = 0;         % Mission start timestamp
        transporter_robots(i).last_update_time = 0;           % Last status update timestamp
        transporter_robots(i).available_time = 0;             % Time when robot becomes available for next task
        
        %% A* path planning results
        transporter_robots(i).current_path = [];              % Current planned path
        transporter_robots(i).path_costs = struct(...         % Path cost breakdown
            'distance_cost', 0, ...
            'time_cost', 0, ...
            'energy_cost', 0, ...
            'total_cost', 0);
    end

    %% Print initialization summary
    fprintf('  Robot initialization completed:\n');
    fprintf('    Collector robots: %d (IDs: %s)\n', ...
            length(collector_robots), ...
            sprintf('%d ', [collector_robots.id]));
    fprintf('    Transporter robots: %d (IDs: %s)\n', ...
            length(transporter_robots), ...
            sprintf('%d ', [transporter_robots.id]));
    
    % Print robot specifications
    fprintf('  Robot specifications:\n');
    fprintf('    Collectors - Speed: %.2fm/s, Energy: %dW(move)/%dW(sense), Range: %dm\n', ...
            collector_robots(1).speed, collector_robots(1).energy_move, ...
            collector_robots(1).energy_sense, collector_robots(1).range_sense);
    fprintf('    Transporters - Speed: %.2fm/s, Energy: %dW(move)/%dW(sense), Range: %dm\n', ...
            transporter_robots(1).speed, transporter_robots(1).energy_move, ...
            transporter_robots(1).energy_sense, transporter_robots(1).range_sense);
    
    fprintf('  All robots positioned at base: [%.1f, %.1f]\n', base_pos(1), base_pos(2));

end