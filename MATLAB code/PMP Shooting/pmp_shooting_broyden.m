function pmp_shooting_broyden
    % Initial guess
    lambda = [-1; -1; -1];  % lambda(0)
    r = shooting_residual(lambda);
    
    % Initial Jacobian approximation using finite difference
    B = finite_jacobian(lambda, 1e-6);
    
    tol = 1e-6;
    max_iter = 30;

    for k = 1:max_iter
        % Solve B * delta = -r
        delta_lambda = -B \ r;
        lambda_new = lambda + delta_lambda;
        
        r_new = shooting_residual(lambda_new);

        % Broyden update
        y = r_new - r;
        s = delta_lambda;
        B = B + ((y - B * s) * s') / (s' * s);

        fprintf('Iter %d: Residual norm = %.4e\n', k, norm(r_new));

        if norm(s) < tol
            lambda = lambda_new;
            break;
        end

        % Update
        lambda = lambda_new;
        r = r_new;
    end

    % Final integration with updated lambda0
    y0 = [0; 0; 0; lambda];
    [t, Y] = ode45(@(t, y) odefun(t, y), linspace(0, 1, 100), y0);

    x = Y(:,1:3);
    lambda_vals = Y(:,4:6);
    u = -0.5 * lambda_vals(:,3);

    % Plot
    figure;
    subplot(3,1,1);
    plot(t, x, 'LineWidth', 2); legend('x_1','x_2','x_3'); title('States');

    subplot(3,1,2);
    plot(t, lambda_vals, '--', 'LineWidth', 2); legend('\lambda_1','\lambda_2','\lambda_3'); title('Costates');

    subplot(3,1,3);
    plot(t, u, 'r-', 'LineWidth', 2); title('Control u(t)');
    xlabel('Time t');
end

function r = shooting_residual(lambda0)
    y0 = [0; 0; 0; lambda0];
    [~, Y] = ode45(@(t, y) odefun(t, y), [0 1], y0);
    x_final = [Y(end, 1) Y(end,5:6)]';
    r = x_final-[1; 0; 0];
end

function J = finite_jacobian(lambda0, h)
    n = length(lambda0);
    J = zeros(3, n);
    for i = 1:n
        d = zeros(n,1); d(i) = h;
        r1 = shooting_residual(lambda0 + d);
        r0 = shooting_residual(lambda0);
        J(:,i) = (r1 - r0) / h;
    end
end

function dydt = odefun(t, y)
    x = y(1:3);
    lambda = y(4:6);
    u = -0.5 * lambda(3);
    
    dx = [x(2); x(3); u];
    dlambda = [0; -lambda(1); -lambda(2)];
    
    dydt = [dx; dlambda];
end
