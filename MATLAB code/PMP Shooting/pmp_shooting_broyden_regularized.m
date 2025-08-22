function pmp_shooting_broyden_regularized
    % Initial guess
    lambda = [-1; -1; -1];
    r = shooting_residual(lambda);
    
    % Initial Jacobian approximation
    B = finite_jacobian(lambda, 1e-6);
    
    % Regularization strength
    alpha = 1e-6;  % Tikhonov regularization parameter

    tol = 1e-6;
    max_iter = 30;

    for k = 1:max_iter
        % Regularized least-squares solve:
        % delta = - (BᵀB + αI)^(-1) * Bᵀ * r
        BTB = B' * B;
        reg_term = alpha * eye(size(BTB));
        delta_lambda = - (BTB + reg_term) \ (B' * r);

        lambda_new = lambda + delta_lambda;
        r_new = shooting_residual(lambda_new);

        % Broyden update
        s = lambda_new - lambda;
        y = r_new - r;
        B = B + ((y - B * s) * s') / (s' * s);

        fprintf('Iter %d: Residual norm = %.4e\n', k, norm(r_new));

        if norm(delta_lambda) < tol
            lambda = lambda_new;
            break;
        end

        lambda = lambda_new;
        r = r_new;
    end

    % Final integration
    y0 = [0; 0; 0; lambda];
    [t, Y] = ode45(@(t, y) odefun(t, y), linspace(0, 1, 100), y0);

    x = Y(:,1:3);
    lambda_vals = Y(:,4:6);
    u = -0.5 * lambda_vals(:,3);

    % Plot
    figure;
    subplot(3,1,1); plot(t, x, 'LineWidth', 2); legend('x_1','x_2','x_3'); title('States');
    subplot(3,1,2); plot(t, lambda_vals, '--', 'LineWidth', 2); legend('\lambda_1','\lambda_2','\lambda_3'); title('Costates');
    subplot(3,1,3); plot(t, u, 'r-', 'LineWidth', 2); title('Control u(t)'); xlabel('Time t');
end

function r = shooting_residual(lambda0)
    y0 = [0; 0; 0; lambda0];
    [~, Y] = ode45(@(t, y) odefun(t, y), [0 1], y0);
    x_final = Y(end, 1:3);
    lambda_final = Y(end, 4:6);

    % Only fix x1(1), and apply transversality for free x2, x3
    r = [x_final(1) - 1; lambda_final(2); lambda_final(3)];
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
