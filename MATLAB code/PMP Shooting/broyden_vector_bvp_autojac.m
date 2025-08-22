function broyden_vector_bvp_autojac
    % Domain and boundary conditions
    a = 0; b = pi;
    alpha = [1; 0];      % y1(0), y2(0)
    beta  = [-1; 2];     % y1(b), y2(b)

    % Initial guess for unknown slopes [v1(0); v2(0)]
    s = [0.0; 0.0];

    % Tolerance and max iterations
    tol = 1e-6;
    max_iter = 30;
    n = length(s);

    % Step size for finite difference
    h = 1e-6;

    % Compute initial residual F(s)
    [~, Y] = ode45(@(x,y) odefun(x, y), [a b], [alpha; s]);
    F = Y(end,1:2)' - beta;

    % Automatic Jacobian estimation via finite difference
    J = zeros(n);
    for j = 1:n
        s_perturb = s;
        s_perturb(j) = s_perturb(j) + h;
        [~, Yh] = ode45(@(x,y) odefun(x, y), [a b], [alpha; s_perturb]);
        Fh = Yh(end,1:2)' - beta;
        J(:,j) = (Fh - F) / h;
    end

    for k = 1:max_iter
        % Solve J * delta_s = -F
        delta_s = -J \ F;
        s_new = s + delta_s;

        % Integrate with updated guess
        y0 = [alpha; s_new];
        [~, Y] = ode45(@(x,y) odefun(x, y), [a b], y0);
        F_new = Y(end,1:2)' - beta;

        fprintf('Iter %d: Residual Norm = %.3e\n', k, norm(F_new));

        if norm(F_new) < tol || norm(delta_s) < tol
            fprintf('Converged at iteration %d\n', k);
            break;
        end

        % Good Broyden update
        dF = F_new - F;
        dS = s_new - s;
        J = J + ((dF - J*dS) * dS') / (dS' * dS);

        % Update
        s = s_new;
        F = F_new;
    end

    fprintf('Final initial slopes: v1(0) = %.8f, v2(0) = %.8f\n', s(1), s(2));

    % Final solution
    y0 = [alpha; s];
    [X, Y] = ode45(@(x,y) odefun(x, y), [a b], y0);

    % Plot results
    figure;
    plot(X, Y(:,1), 'b-', 'LineWidth', 2); hold on;
    plot(X, Y(:,3), 'r--', 'LineWidth', 2);
    legend('y_1(x)', 'y_2(x)');
    xlabel('x'); ylabel('y_i(x)');
    title('Vector BVP via Broyden + Shooting + Auto Jacobian');
    grid on;
end

% First-order ODE system
function dydx = odefun(x, y)
    dydx = zeros(4,1);
    y1 = y(1); v1 = y(3);
    y2 = y(2); v2 = y(4);

    dydx(1) = v1;                        % y1'
    dydx(2) = v2;                        % y2'
    dydx(3) = -x * sin(y1) + y2;         % v1'
    dydx(4) = y1^2 - cos(x);             % v2'
end
