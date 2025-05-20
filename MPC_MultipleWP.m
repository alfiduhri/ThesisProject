import casadi.*

% Problem setup
N = 20;                         % MPC horizon
g_val = 9.81;
b_gamma=1.5; b_va=0.6; b_phi=7;
b=[b_gamma;b_va;b_phi];
waypoints = [0, 0, 20; 10, 10, 16; 15,12,20;20,12,20];
nw = size(waypoints, 1);
wp_idx = round(linspace(1, N, nw));

nx = 7;   % State: [pn; pe; h; chi; gamma; Va; phi]
nu = 3;   % Control: [gamma_c; Va_c; phi_c]

% Symbolic variables
X = SX.sym('X', nx, N+1);
U = SX.sym('U', nu, N);
T = SX.sym('T');
dt = T/N;

x = SX.sym('x', nx);
u = SX.sym('u', nu);

xdot = [x(6)*cos(x(4))*cos(x(5));
        x(6)*sin(x(4))*cos(x(5));
        x(6)*sin(x(5));
        g_val/x(6)*tan(x(7));
        b(1)*(u(1)-x(5));
        b(2)*(u(2)-x(6));
        b(3)*(u(3)-x(7))];
f = Function('f', {x, u}, {xdot});

% Cost and constraints
J = 0; g_constr = [];

lambda_wp = 20;
lambda_time = 10;

lbw_u = [-deg2rad(20); -inf; -deg2rad(60)];
ubw_u = [ deg2rad(45);  inf;  deg2rad(60)];

for k = 1:N
    x_next = X(:,k) + dt * f(X(:,k), U(:,k));
    g_constr = [g_constr; X(:,k+1) - x_next];
    J = J + U(:,k)' * U(:,k) * dt;
end

for i = 1:nw
    wp_k = wp_idx(i);
    wp = waypoints(i,:)';
    pos = X(1:3, wp_k);
    J = J + lambda_wp * sumsqr(pos - wp);
end

J = J + lambda_time * T;

% Placeholder for initial and terminal constraints
x0_sym = SX.sym('x0', nx);
g_constr = [X(:,1) - x0_sym; g_constr];

% Decision variables
w = [X(:); U(:); T];
nlp = struct('x', w, 'f', J, 'g', g_constr, 'p', x0_sym);
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
solver = nlpsol('solver', 'ipopt', nlp, opts);

% Initial and simulation setup
x0 = [waypoints(1,:)'; 0; 0; 5; 0];
T_init = 10;

T_min = 5; T_max = 20;
lbw = [-inf*ones(nx*(N+1),1); repmat(lbw_u, N, 1); T_min];
ubw = [ inf*ones(nx*(N+1),1); repmat(ubw_u, N, 1); T_max];

% MPC simulation
sim_time = 20; dt_sim = 0.5;
sim_steps = sim_time / dt_sim;
x_sim = zeros(nx, sim_steps+1);
u_sim = zeros(nu, sim_steps);
x_sim(:,1) = x0;

% Initial guess
X0_guess = repmat(x0, 1, N+1);
U0_guess = zeros(nu, N);
w0 = [X0_guess(:); U0_guess(:); T_init];
lbg = zeros(size(g_constr));
ubg = zeros(size(g_constr));

for t = 1:sim_steps
    % Solve the MPC problem
    sol = solver('x0', w0, 'lbg', lbg, 'ubg', ubg, 'lbx', lbw, 'ubx', ubw, 'p', x_sim(:,t));
    w_opt = full(sol.x);
    
    X_opt = reshape(w_opt(1:(nx*(N+1))), nx, N+1);
    U_opt = reshape(w_opt((nx*(N+1)+1):(end-1)), nu, N);
    T_opt = w_opt(end);

    % Apply the first control input
    u_sim(:,t) = U_opt(:,1);
    x_sim(:,t+1) = x_sim(:,t) + dt_sim * full(f(x_sim(:,t), u_sim(:,t)));

    % Shift the guess for warm start
    X0_guess = [X_opt(:,2:end), X_opt(:,end)];
    U0_guess = [U_opt(:,2:end), U_opt(:,end)];
    w0 = [X0_guess(:); U0_guess(:); T_opt];
end

% Plot results
figure;
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'b-o'); hold on;
plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('x'); ylabel('y'); zlabel('h');
title('UAV Trajectory (MPC)');
legend('MPC Trajectory', 'Waypoints'); grid on;
