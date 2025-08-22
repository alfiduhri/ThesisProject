W = [
    0  0  20;
    5  0  20;
    10 -5 20;
    15 0  20;
    10 5  20;
    5  0  20;
    5  10 20;
    10 15 20;
    15 20 20;
    20 20 20;
]; 
pruned = prune_loops_exact(W);
scatter(W(:,1),W(:,2),'red','filled');
figure
scatter(pruned(:,1),pruned(:,2),'blue','filled');


function pruned = prune_loops_exact(waypoints)
    pruned = [];
    visited = containers.Map();  % hash map for quick lookup
    index = 1;
    
    for i = 1:size(waypoints, 1)
        key = mat2str(waypoints(i, :));  % convert row to string as key
        
        if isKey(visited, key)
            % Loop detected; truncate path to the earlier occurrence
            % index acts as a value in hash map
            index = visited(key) + 1;
            pruned = pruned(1:visited(key), :);
        else
            visited(key) = index; % This is to assign value and key into map
            pruned = [pruned; waypoints(i, :)]; %#ok<AGROW>
            index = index + 1;
        end
    end
end
