[path,simplified]=custom_path_with_loop_removal();

function [path,simplified]=custom_path_with_loop_removal()
    %% 1. Create custom path with: straight → left turn → loop → straight → right turn
    path = [];

    % (a) Straight line
    straight1 = [linspace(0, 10, 10)', zeros(10,1)];
    path = [path; straight1];

    % (b) Slight left turn (a quarter arc upward)
    theta1 = linspace(0, pi/6, 10);
    r1 = 5;
    center1 = [10, 0];
    left_arc = [center1(1) + r1*sin(theta1)', center1(2) + r1*(1 - cos(theta1))'];
    path = [path; left_arc];

    % (c) Loop (small full circle)
    theta2 = linspace(0, 2*pi, 20);
    r2 = 2;
    loop_center = [14, 3];
    loop = [loop_center(1) + r2*cos(theta2)', loop_center(2) + r2*sin(theta2)'];
    path = [path; loop];

    % (d) Straight again
    straight2 = [linspace(16, 26, 10)', ones(10,1)*3];
    path = [path; straight2];

    % (e) Slight right turn (downward)
    theta3 = linspace(0, pi/6, 10);
    r3 = 5;
    center3 = [26, 3];
    right_arc = [center3(1) + r3*sin(theta3)', center3(2) - r3*(1 - cos(theta3))'];
    path = [path; right_arc];

    %% 2. Apply loop removal
    maxSkipDist = 2;
    simplified = removeLoopsDijkstra(path, maxSkipDist);

    %% 3. Plot results
    figure; hold on; grid on; axis equal;
    plot(path(:,1), path(:,2), 'b--o', 'DisplayName', 'Original Path');
    plot(simplified(:,1), simplified(:,2), 'r-o', 'LineWidth', 2, 'DisplayName', 'Simplified Path');
    legend;
    title('Loop Removal (Dijkstra Optimization)');
end

%% Dijkstra-based Loop Remover
function simplifiedPath = removeLoopsDijkstra(path, maxSkipDist)
    n = size(path, 1);
    dist = inf(1, n);
    prev = NaN(1, n);
    dist(1) = 0;

    Q = [1];

    while ~isempty(Q)
        [~, idx] = min(dist(Q));
        u = Q(idx);
        Q(idx) = [];

        for v = (u+1):n
            d = norm(path(u,:) - path(v,:));
            if d <= maxSkipDist
                alt = dist(u) + d;
                if alt < dist(v)
                    dist(v) = alt;
                    prev(v) = u;
                    if ~ismember(v, Q)
                        Q = [Q, v];
                    end
                end
            end
        end
    end

    % Reconstruct path
    simplifiedPath = [];
    u = n;
    while ~isnan(u)
        simplifiedPath = [path(u, :); simplifiedPath];
        u = prev(u);
    end
end
