clear; clc; close all;

% === UAV Turning Radius Constraint ===
Rmin = 20;

% === Define Original Waypoints (x, y, heading in degrees) ===
% Contains a loop in middle
waypoints = [
    0   0    0;
    20  0    0;
    40 10   45;
    30 30   90;
    10 20  180;
    0   0  270;
    60 60   45
];

% Convert heading to radians
waypoints(:,3) = deg2rad(waypoints(:,3));

% === Setup Dubins Connector ===
connector = dubinsConnection('MinTurningRadius', Rmin);

% === Loop Removal ===
simplified_idx = [1]; % always keep first

i = 1;
while i < size(waypoints, 1)
    found = false;
    % Try to jump to the farthest reachable waypoint
    for j = size(waypoints,1):-1:(i+1)
        start_pose = waypoints(i, :);
        end_pose = waypoints(j, :);

        [path_obj, ~] = connect(connector, start_pose, end_pose);
        if isempty(path_obj)
            continue; % infeasible
        end

        % Compute original distance through intermediate waypoints
        inter_dist = 0;
        for k = i:(j-1)
            dp = norm(waypoints(k+1,1:2) - waypoints(k,1:2));
            inter_dist = inter_dist + dp;
        end

        dubins_length = path_obj{1}.Length;

        % If Dubins path is significantly shorter (tune this threshold)
        if dubins_length + 5 < inter_dist
            simplified_idx(end+1) = j;
            i = j; % jump forward
            found = true;
            break;
        end
    end

    if ~found
        % No valid skip found; include next
        i = i + 1;
        simplified_idx(end+1) = i;
    end
end

% === Extract simplified waypoints ===
simplified_wps = waypoints(simplified_idx, :);

% === Plot Results ===
figure; hold on; axis equal; grid on;
title('Loop Removal with Dubins Constraint');
xlabel('X (m)'); ylabel('Y (m)');

% Plot original path
plot(waypoints(:,1), waypoints(:,2), 'r--o', 'DisplayName', 'Original Path');

% Plot simplified path
for k = 1:(length(simplified_idx)-1)
    s = simplified_wps(k, :);
    e = simplified_wps(k+1, :);
    [path, ~] = connect(connector, s, e);
    states = interpolate(path{1}, linspace(0, path{1}.Length, 100));
    plot(states(:,1), states(:,2), 'b-', 'LineWidth', 2);
end

% Mark simplified waypoints
plot(simplified_wps(:,1), simplified_wps(:,2), 'bo', 'MarkerSize', 8, 'DisplayName', 'Simplified Path');

legend();
