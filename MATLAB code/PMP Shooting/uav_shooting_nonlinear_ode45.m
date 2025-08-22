function uav_shooting_nonlinear_ode45
    % Time and waypoint setup
    T = 15;
    t_wp = [5, 10];
    wp_times = [0, t_wp, T];

    % Waypoints [x; y; h]
    wp_pos = [0 0 0;
              10 5 2;
              20 10 5;
              30 15 8]';

    % Control bounds: [min, max] for each control input
    u_bounds = [-2 2;    % u_v
                -1 1;    % u_theta
                -1 1];   % u_h

    rho = [10, 10];      % Penalty weights for waypoints

    % Initial guess for costate [lambda_x, lambda_y, lambda_h, lambda_v, lambda_theta]
    lambda0 = zeros(5,1);

    % Newton shooting loop
    for iter = 1:15
        [xtraj, x_wp, lambdas] = simulate_nonlinear_ode(lambda0, wp_times, wp_pos, rho, u_bounds);

        % Compute error (only position/altitude) at interior waypoints
        err = [];
        for i = 2:length(wp_times)-1
            err = [err; x_wp{i}(1:3) - wp_pos(:, i)];
        end

        % Jacobian estimation (finite difference)
        J = zeros(length(err), 5);
        delta = 1e-4;
        for j = 1:5
            lambda0_pert = lambda0;
            lambda0_pert(j) = lambda0_pert(j) + delta;
            [~, x_wp_d] = simulate_nonlinear_ode(lambda0_pert, wp_times, wp_pos, rho, u_bounds);
            err_d = [];
            for i = 2:length(wp_times)-1
                err_d = [err_d; x_wp_d{i}(1:3) - wp_pos(:, i)];
            end
            J(:, j) = (err_d - err) / delta;
        end

        % Newton update
        delta_lambda = -J \ err;
        lambda0 = lambda0 + delta_lambda;

        fprintf('Iter %d, Error norm: %.6f\n', iter, norm(err));
        if norm(err) < 1e-3
            break;
        end
    end

    % Final trajectory with converged lambda0
    [xtraj, ~] = simulate_nonlinear_ode(lambda0, wp_times, wp_pos, rho, u_bounds);

    % Plot trajectory
    figure;
    plot3(xtraj(1,:), xtraj(2,:), xtraj(3,:), 'b', 'LineWidth', 2); hold on;
    plot3(wp_pos(1,:), wp_pos(2,:), wp_pos(3,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    xlabel('x'); ylabel('y'); zlabel('h'); title('UAV Trajectory using ode45');
    legend('Trajectory','Waypoints'); grid on; view(3);
end
function [xtraj, x_wp, lambdas] = simulate_nonlinear_ode(lambda0, wp_times, wp_pos, rho, u_bounds)
    x0 = [wp_pos(:,1); 3; 0];  % Initial state: position + speed=3 + heading=0
    T = wp_times(end);

    x_wp = cell(1, length(wp_times));
    lambdas = {};
    x_all = [];
    t_all = [];

    lambda = lambda0;
    t_start = 0;
    x_start = x0;

    for i = 2:length(wp_times)
        t_end = wp_times(i);
        tspan = [t_start t_end];

        % ODE45 with current costate
        odeopts = odeset('RelTol',1e-6,'AbsTol',1e-9);
        [t_seg, x_seg] = ode45(@(t, x) uav_dynamics(t, x, lambda, u_bounds), tspan, x_start, odeopts);

        x_all = [x_all; x_seg];
        t_all = [t_all; t_seg];

        x_wp{i} = x_seg(end, :)';  % Save final state of this segment
        x_start = x_seg(end, :)';  % Set initial condition for next segment

        if i < length(wp_times)
            lambda(1:3) = lambda(1:3) - rho(i-1) * (x_wp{i}(1:3) - wp_pos(:,i));  % Costate jump
            lambdas{i} = lambda;
        end
        t_start = t_end;
    end

    x_wp{1} = x0;
    x_wp{end} = x_all(end,:)';

    xtraj = x_all';  % Transpose to return [states x time]
end
function dx = uav_dynamics(~, x, lambda, u_bounds)
    % State: x = [x; y; h; v; theta]
    % Costate: lambda = [lambda_x, lambda_y, lambda_h, lambda_v, lambda_theta]

    % Compute control from PMP and clip
    u_v     = -lambda(4);
    u_theta = -lambda(5);
    u_h     = -lambda(3);

    u = [u_v; u_theta; u_h];
    u = max(min(u, u_bounds(:,2)), u_bounds(:,1));

    dx = zeros(5,1);
    dx(1) = x(4) * cos(x(5));   % x_dot
    dx(2) = x(4) * sin(x(5));   % y_dot
    dx(3) = u(3);               % h_dot
    dx(4) = u(1);               % v_dot
    dx(5) = u(2);               % theta_dot
end
