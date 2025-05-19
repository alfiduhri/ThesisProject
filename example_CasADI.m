clear; clc; close all
import casadi.*

% Problem setup
N = 600;                   % Number of control intervals
waypoints = [2, 2; 5, 5; 8, 1;10,3];  % 3 waypoints [x, y]
nw = size(waypoints, 1);
wp_idx = round(linspace(1, N, nw));  % Time indices to associate waypoints

nx = 4;   % State: [x; y; vx; vy]
nu = 2;   % Control: [ax; ay]

% Symbolic variables
X = SX.sym('X', nx, N+1);
U = SX.sym('U', nu, N);
T = SX.sym('T');                 % Total time is now a variable
dt = T/N;

% Dynamics: ẋ = [vx; vy; ax; ay]
x = SX.sym('x', nx);
u = SX.sym('u', nu);
xdot = [x(3); x(4); u(1); u(2)];
f = Function('f', {x, u}, {xdot});

% Cost and constraints
J = 0;
g = [];

% Parameters
lambda_wp = 50;
lambda_length = 1;
loop_penalty = 1;
lambda_time = 10;              % Penalize total time

for k = 1:N
    % Dynamics constraint
    x_next = X(:,k) + (T/N) * f(X(:,k), U(:,k));
    g = [g; X(:,k+1) - x_next];

    % Control effort cost
    J = J + U(:,k)' * U(:,k) * (T/N);

    % % Path length penalty
    % if k < N
    %     pos_k = X(1:2, k);
    %     pos_k1 = X(1:2, k+1);
    %     J = J + lambda_length * sumsqr(pos_k1 - pos_k);
    % end
    % 
    % % Backward motion penalty
    % if k > 1
    %     delta_x = X(1,k) - X(1,k-1);
    %     delta_y = X(2,k) - X(2,k-1);
    %     J = J + loop_penalty * fmax(0, -delta_x)^2;
    %     J = J + loop_penalty * fmax(0, -delta_y)^2;
    % end
end

% Soft waypoint penalties
for i = 1:nw
    wp_k = wp_idx(i);
    wp = waypoints(i,:)';
    pos = X(1:2, wp_k);
    J = J + lambda_wp * sumsqr(pos - wp);
end

% Penalize total time (optional: or minimize just T)
J = J + lambda_time * T;

% Initial and final state constraints
x0 = [0; 0; 0; 0];
xf = [waypoints(end,:)'; 0; 0];
g = [g; X(:,1) - x0];
g = [g; X(:,end) - xf];

% Pack decision variables
w = [X(:); U(:); T];
nlp = struct('x', w, 'f', J, 'g', g);

% Solver options
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Initial guess
w0 = [zeros(nx*(N+1)+nu*N,1); 10];  % guess T = 10

% Bounds on constraints
lbg = zeros(size(g));
ubg = zeros(size(g));

% Bounds on variables
T_min = 5; T_max = 20;
lbw = [-inf*ones(nx*(N+1)+nu*N,1); T_min];
ubw = [ inf*ones(nx*(N+1)+nu*N,1); T_max];

% Solve
sol = solver('x0', w0, 'lbg', lbg, 'ubg', ubg, 'lbx', lbw, 'ubx', ubw);
w_opt = full(sol.x);
T_opt = w_opt(end);

% Extract solution
X_opt = reshape(w_opt(1:(nx*(N+1))), nx, N+1);
U_opt = reshape(w_opt((nx*(N+1)+1):(end-1)), nu, N);

% Plot trajectory
figure;
plot(X_opt(1,:), X_opt(2,:), 'b-o'); hold on;
plot(waypoints(:,1), waypoints(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('x'); ylabel('y'); title(sprintf('UAV Trajectory (Optimized T = %.2f s)', T_opt));
legend('Trajectory', 'Waypoints'); grid on;
