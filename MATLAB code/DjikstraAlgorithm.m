% Original path (with a visible loop)
% path = [0 0;
%         1 1;
%         2 2;
%         3 1;
%         2 0;
%         1 -1;
%         0 0;    % back to start — loop
%         1 1;
%         2 2;
%         3 3];
path = [
    0,  0,  20;
    5,  0,  20;
    10, -5, 20;
    15, 0,  20;
    10, 5,  20;
    5,  0,  20;
    10, -5, 20;
    15, 0,  20;
    10, 5,  20;
    5,  10, 20;
    10, 15, 20;
    15, 20, 20;
    20, 20, 20;
];

maxSkipDist = 10; % Max allowed shortcut distance
simplified = removeLoopsDijkstra(path, maxSkipDist);

% Plot original vs simplified
figure
plot3(path(:,1), path(:,2), path(:,3),'b--o', 'DisplayName', 'Original Path');
hold on
grid on
plot3(simplified(:,1), simplified(:,2),simplified(:,3),...
    'r-o', 'LineWidth', 2, 'DisplayName', 'Simplified Path');
legend;
xlabel('x (m)');
ylabel('y(m)');
zlabel('h(m)');




function simplifiedPath = removeLoopsDijkstra(path, maxSkipDist)
    n = size(path, 1);
    dist = inf(1, n);
    prev = NaN(1, n);
    dist(1) = 0;

    % Priority queue as a list (Dijkstra)
    Q = [1];

    while ~isempty(Q)
        % Pop node with minimum dist
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

    % Reconstruct shortest path
    simplifiedPath = [];
    u = n;
    while ~isnan(u)
        simplifiedPath = [path(u, :); simplifiedPath];
        u = prev(u);
    end
end
