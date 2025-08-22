clear; clc; close all;
import casadi.*

% Parameters
N = 800;  % discretization steps
waypoints = [0, 0;
             20, 10;
             40, 0;
             60,5;
             40,20;
             60,10];
n_wp = size(waypoints, 1);

% Define symbols
x = MX.sym('x'); y = MX.sym('y'); theta = MX.sym('theta');
v = MX.sym('v'); omega = MX.sym('omega');
state = [x; y; theta];
control = [v; omega];

% Dynamics function
xdot = [v*cos(theta);
        v*sin(theta);
        omega];
f = Function('f', {state, control}, {xdot});

% Decision variables
X = MX.sym('X', 3, N+1);  % states over time
U = MX.sym('U', 2, N);    % controls over time
T = MX.sym('T', 1);       % final time

% Time step (normalized)
dt = 1.0 / N;

% Build constraints and Control cost
g = {};
g{end+1} = X(:,1) - [waypoints(1,:)'; 0];  % initial state
Jc = 0;

for k = 1:N
    x_next = X(:,k) + T * dt * f(X(:,k), U(:,k));
    g{end+1} = X(:,k+1) - x_next;

    Jc = Jc + T * dt * (U(1,k)^2 + U(2,k)^2);
end

% Waypoint constraints (x, y only)
wp_indices = round(linspace(1, N+1, n_wp));
lambda_wp = 1;
J = 0;
for i = 2:n_wp
    % wp_k = wp_indices(i);
    % wp = waypoints(i,:)';
    % pos = X(1:2, wp_k);
    % J=J+lambda_wp * sumsqr(pos - wp);
    g{end+1} = X(1:2, wp_indices(i)) - waypoints(i,:)';
end

% Weights
alpha = 1;   % weight for time
beta = 0.1;  % weight for control effort

% Combined objective
J = J+alpha * T + beta * Jc;

% All variables flattened
w = [X(:); U(:); T];

% Bounds and initial guess
w0 = zeros(size(w));
lbx = -inf(size(w));
ubx = inf(size(w));

% Control bounds
for k = 1:N
    base = numel(X(:)) + (k-1)*2;
    lbx(base + 1) = 0;   % v >= 0
    ubx(base + 1) = 2;   % v <= 2
    lbx(base + 2) = -1;  % omega
    ubx(base + 2) = 1;
end

% T bounds
lbx(end) = 1;
ubx(end) = 100;
w0(end) = 10;

% Constraint bounds (all equality)
g_expr = vertcat(g{:});
nlp = struct('x', w, 'f', J, 'g', g_expr);

% Solver
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
solver = nlpsol('solver', 'ipopt', nlp, opts);

sol = solver('x0', w0, 'lbx', lbx, 'ubx', ubx, 'lbg', 0, 'ubg', 0);
w_opt = full(sol.x);

% Extract solution
X_opt = reshape(w_opt(1:3*(N+1)), 3, N+1);
T_opt = w_opt(end);

% Plot results
figure; hold on; grid on; axis equal;
plot(X_opt(1,:), X_opt(2,:), 'b-', 'LineWidth', 2);
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
title(['Optimal Path (T = ' num2str(T_opt, '%.2f') ' s)']);
xlabel('x'); ylabel('y');
legend('Trajectory', 'Waypoints');
