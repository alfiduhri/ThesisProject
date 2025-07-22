% Define waypoints
waypoints = [
    0,  0,  20;
    5,  0,  20;
    10, -5, 20;
    15, 0,  20;
    10, 5,  20;
    5,  10, 20;
    10, 15, 20;
    15, 20, 20;
    20, 20, 20;
];
pruned = prune_loops_approx(W, 5);
% Result: [0 0; 1 1; 3 3]
scatter(W(:,1),W(:,2),'red','filled');
figure
scatter(pruned(:,1),pruned(:,2),'blue','filled');

function pruned = prune_loops_approx(waypoints, threshold)
    if nargin < 2 % this is to check whether we have threshold or not
        threshold = 1.0; % default distance threshold
    end

    pruned = waypoints(1, :);
    
    for i = 2:size(waypoints, 1)
        wp = waypoints(i, :);
        loop_found = 0;
        
        for j = 1:size(pruned, 1)
            dist = norm(wp - pruned(j, :));
            if dist < threshold
                % Loop detected: truncate to that point
                pruned = pruned(1:j, :);
                loop_found = 1;
                break;
            end
        end
        
        if ~loop_found
            pruned = [pruned; wp]; %#ok<AGROW>
        end
    end
end
