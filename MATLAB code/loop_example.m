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

% Extract X, Y, Z coordinates
X = waypoints(:,1);
Y = waypoints(:,2);
Z = waypoints(:,3);

figure; hold on; grid on;

% Plot full path in blue
plot3(X, Y, Z, '-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', 'b');

% Highlight loop segment (waypoints 2 to 6) in green with thicker line
loop_indices = [2:5 2];
plot3(X(loop_indices), Y(loop_indices), Z(loop_indices), '-o', ...
    'Color', [0 0.7 0], 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Labels and formatting
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Waypoints with Highlighted Loop');
axis equal;
view(3);

% Label all waypoints
for i = 1:length(X)
    text(X(i), Y(i), Z(i), sprintf(' %d', i), 'FontSize', 10, 'Color', 'k');
end

hold off;
