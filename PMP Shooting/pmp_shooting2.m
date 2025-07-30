function pmp_shooting_3state
    % Initial guess for costates
    lambda0 = [-1; -1; -1];  % Initial guess: lambda(0)
    tol = 1e-6;
    max_iter = 20;

    for k = 1:max_iter
        % Residual and Jacobian (approximate)
        r = shooting_residual(lambda0);
        J = finite_jacobian(lambda0, 1e-6);

        % Newton step: solve J * delta = -r
        delta = -J \ r;
        lambda0_new = lambda0 + delta;

        fprintf('Iter %d: Residual norm = %.4e\n', k, norm(r));

        if norm(delta) < tol
            break;
        end

        lambda0 = lambda0_new;
    end

    % Final integration
    y0 = [0; 0; 0; lambda0];  % x(0), lambda(0)
    [t, Y] = ode45(@(t, y) odefun(t, y), linspace(0, 1, 100), y0);

    x = Y(:,1:3);
    lambda = Y(:,4:6);
    u = -0.5 * lambda(:,3);

    % Plot
    figure;
    subplot(3,1,1);
    plot(t, x, 'LineWidth', 2); legend('x_1','x_2','x_3'); title('States');

    subplot(3,1,2);
    plot(t, lambda, '--', 'LineWidth', 2); legend('\lambda_1','\lambda_2','\lambda_3'); title('Costates');

    subplot(3,1,3);
    plot(t, u, 'r-', 'LineWidth', 2); title('Control u(t)');
    xlabel('Time t');
end

function r = shooting_residual(lambda0)
    y0 = [0; 0; 0; lambda0];  % Initial state and costates
    [~, Y] = ode45(@(t, y) odefun(t, y), [0 1], y0);
    x_final = Y(end, 1:3)';
    r = x_final - [1; 0; 0];  % Target x(1)
end

function J = finite_jacobian(lambda0, h)
    % Finite-difference approximation of Jacobian
    n = length(lambda0);
    J = zeros(3, n);
    for i = 1:n
        d = zeros(n,1);
        d(i) = h;
        r1 = shooting_residual(lambda0 + d);
        r0 = shooting_residual(lambda0);
        J(:,i) = (r1 - r0) / h;
    end
end

function dydt = odefun(t, y)
    x = y(1:3);
    lambda = y(4:6);
    u = -0.5 * lambda(3);
    
    dx1 = x(2);
    dx2 = x(3);
    dx3 = u;

    dl1 = 0;
    dl2 = -lambda(1);
    dl3 = -lambda(2);

    dydt = [dx1; dx2; dx3; dl1; dl2; dl3];
end
