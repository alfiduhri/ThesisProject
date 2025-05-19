clear; clc; close all
import casadi.*

% Problem setup
N = 300;                   % Number of control intervals
g = 9.81;                % Gravity acceleration
b_gamma=0.5;
b_va=0.5;
b_phi=0.5;
b=[b_gamma;b_va;b_phi];  % b=[b_gamma;b_va;b_phi]
waypoints = [0, 0, 20; 15, 15, 20; 30,12,20;30,30,20];  % 3 waypoints [x, y]
nw = size(waypoints, 1);
wp_idx = round(linspace(1, N, nw));  % Time indices to associate waypoints

nx = 7;   % State: [pn; pe; h; chi; gamma; Va; phi]
nu = 3;   % Control: [gamma_c; Va_c; phi_c]

% Symbolic variables for discretization
X = SX.sym('X', nx, N+1);
U = SX.sym('U', nu, N);
T = SX.sym('T');                 % Total time is now a variable
dt = T/N;

% Dynamics: ẋ = [vx; vy; ax; ay]
x = SX.sym('x', nx);
u = SX.sym('u', nu);
xdot=[x(6)*cos(x(4))*cos(x(5));...
    x(6)*sin(x(4))*cos(x(5));...
    x(6)*sin(x(5));...
    g/x(6)*tan(x(7));...
    b(1)*(u(1)-x(5));...
    b(2)*(u(2)-x(6));...
    b(3)*(u(3)-x(7))];
f = Function('f', {x, u}, {xdot});

% Cost and constraints
J = 0;
g = [];

% Parameters
lambda_wp = 50;
lambda_time = 10;              % Penalize total time

for k = 1:N
    % Dynamics constraint
    x_next = X(:,k) + (T/N) * f(X(:,k), U(:,k));
    g = [g; X(:,k+1) - x_next];

    % Control effort cost
    J = J + U(:,k)' * U(:,k) * (T/N);
end

% Soft waypoint penalties
for i = 1:nw
    wp_k = wp_idx(i);
    wp = waypoints(i,:)';
    pos = X(1:3, wp_k);
    J = J + lambda_wp * sumsqr(pos - wp);
end

% Penalize total time (optional: or minimize just T)
J = J + lambda_time * T;

% Initial and final state constraints
x0=[0;0;20;0;0;5;0];
xf = waypoints(end,:)';
g = [X(:,1) - x0;g];
g = [g; X(1:3,end) - xf];

% Pack decision variables
w = [X(:); U(:); T];
nlp = struct('x', w, 'f', J, 'g', g);

% Solver options
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Initial guess
w0 = [ones(nx*(N+1)+nu*N,1); 10];  % guess T = 10

% Bounds on constraints
lbg = zeros(size(g));
ubg = zeros(size(g));

% Bounds on variables
T_min = 5; T_max = 30;
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
plot3(X_opt(1,:), X_opt(2,:), X_opt(3,:), 'b-o'); hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('x'); ylabel('y'); zlabel('h');
title(sprintf('UAV Trajectory (Optimized T = %.2f s)', T_opt));
legend('Trajectory', 'Waypoints'); grid on;
