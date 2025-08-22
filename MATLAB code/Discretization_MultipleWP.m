clear; clc; close all
import casadi.*

% Problem setup
N = 100;                   % Number of control intervals
g = 9.81;                % Gravity acceleration
b_gamma=1.5;
b_va=0.6;
b_phi=7;
b=[b_gamma;b_va;b_phi];  % b=[b_gamma;b_va;b_phi]
waypoints = [
    0 0 20;
    5 0 20;
    10 5 16;
    10 15 20;
    15 20 20;
    20 20 20;
];  % 3 waypoints [x, y, z]
% waypoints = [
% 0	0	20;
% 5	0	20;
% 10	5	20;
% 5	10	20;
% 10	15	20;
% 15	20	20;
% 20	20	20;
% ]; 
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
% State: [pn; pe; h; chi; gamma; Va; phi]
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
lambda_wp = 20;
lambda_time = 10;              % Penalize total time
lbw = [];
ubw = [];
for k = 1:N
    % Dynamics constraint
    x_next = X(:,k) + (T/N) * f(X(:,k), U(:,k));
    g = [g; X(:,k+1) - x_next];

    % Control effort cost
    J = J + U(:,k)' * U(:,k) * (T/N);
    lbw=[lbw;-deg2rad(30);5;-deg2rad(60)];
    ubw=[ubw;deg2rad(20);15;deg2rad(60)];
end

% Soft waypoint penalties
for i = 1:nw
    wp_k = wp_idx(i);
    wp = waypoints(i,:)';
    pos = X(1:3, wp_k);
    J = J + lambda_wp * sumsqr(pos - wp)^2;
end

% Penalize total time (optional: or minimize just T)
J = J + lambda_time * T;

% Initial and final state constraints
x0=[waypoints(1,:)';deg2rad(0);0;5;0];
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
lbw = [-inf*ones(nx*(N+1),1);lbw; T_min];
ubw = [inf*ones(nx*(N+1),1);ubw; T_max];

% Solve
sol = solver('x0', w0, 'lbg', lbg, 'ubg', ubg, 'lbx', lbw, 'ubx', ubw);
w_opt = full(sol.x);
T_opt = w_opt(end);

% Extract solution
X_opt = reshape(w_opt(1:(nx*(N+1))), nx, N+1);
X_opt = round(X_opt,2);
U_opt = reshape(w_opt((nx*(N+1)+1):(end-1)), nu, N);

% Plot trajectory
figure;
plot3(X_opt(1,:), X_opt(2,:), X_opt(3,:), 'b-o'); hold on;
loc = 2:length(waypoints)-1;
plot3(waypoints(loc,1), waypoints(loc,2), waypoints(loc,3), 'rx',...
    'MarkerSize', 10, 'LineWidth', 2);
scatter3(waypoints(1,1),waypoints(1,2),waypoints(1,3),'d','filled',...
    'MarkerFaceColor',"#EDB120",'MarkerEdgeColor',"#EDB120",...
    "LineWidth",2);
scatter3(waypoints(end,1),waypoints(end,2),waypoints(end,3),'s','filled',...
    'MarkerFaceColor',"#A2142F",'MarkerEdgeColor',"#A2142F",...
    "LineWidth",2);
xlabel('x'); ylabel('y'); zlabel('h');
title(sprintf('UAV Trajectory (Optimized T = %.2f s)', T_opt));
legend('Trajectory', 'Waypoints','Start','Finish'); grid on;

% Plot the control
k = 0:N-1;              % Discrete time indices
Ts = T_opt/N;              % Sampling time
T_all = k * Ts;            % Map discrete time to continuous time

figure
subplot(2,1,1);
stairs(T_all,U_opt(2,:),'LineWidth',1)
ylabel('Control input V_a (m/s)')
title('Time history of V_a command')
grid on

subplot(2,1,2); 
stairs(T_all,rad2deg(U_opt(1,:)),'-.','LineWidth',1.5)
hold on
stairs(T_all,rad2deg(U_opt(3,:)),'--*','LineWidth',1.5)
xlabel('Time (s)')
title('Time history of \gamma_c and \phi_c')
ylabel('Control Inputs (deg)')
grid on
legend('\gamma_c','\phi_c')

%Plot the velocity, roll angle, course angle, flight path angle
%State: [pn; pe; h; chi; gamma; Va; phi]
kk=0:N;
T_all2 = kk * Ts;  
figure
subplot(4,1,1)
plot(T_all2,X_opt(6,:),'LineWidth',1.5)
ylabel('Velocity (m/s)')
title('Time history of the velocity, \gamma, \psi, and \phi')
grid on

subplot(4,1,2)
plot(T_all2,rad2deg(X_opt(5,:)),'LineWidth',1.5)
ylabel('\gamma (deg)')
grid on

subplot(4,1,3)
plot(T_all2,rad2deg(X_opt(4,:)),'LineWidth',1.5)
ylabel('\psi (deg)')
grid on

subplot(4,1,4)
plot(T_all2,rad2deg(X_opt(7,:)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\phi (deg)')
grid on
