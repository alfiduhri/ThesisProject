function broyden_shooting_bvp_clean

    % Problem setup
    a = 0; b = pi;
    alpha = 1;           % y(a)
    beta = -1;           % y(b)
    s0 = 0.0;            % Initial guess for y'(a)
    tol = 1e-6;
    max_iter = 20;

    % Initial slope guess and finite difference step
    h = 1e-6;

    % Evaluate F(s0)
    [~, Y0] = ode45(@(x, y) odefun(x, y), [a b], [alpha; s0]);
    F0 = Y0(end, 1) - beta;

    % Estimate Jacobian using finite difference
    [~, Y1] = ode45(@(x, y) odefun(x, y), [a b], [alpha; s0 + h]);
    F1 = Y1(end, 1) - beta;
    J = (F1 - F0) / h;

    % Initialize
    s = s0;
    F = F0;

    for k = 1:max_iter
        ds = -F / J;
        s_new = s + ds;

        % Single ODE integration per iteration
        y0 = [alpha; s_new];
        [~, Y] = ode45(@(x, y) odefun(x, y), [a b], y0);
        F_new = Y(end, 1) - beta;

        dS = s_new - s;
        dF = F_new - F;

        fprintf('Iter %d: s = %.8f, Residual = %.3e\n', k, s_new, F_new);

        if abs(F_new) < tol || abs(dS) < tol
            fprintf('Converged at iteration %d\n', k);
            break;
        end

        % Broyden update
        J = J + (dF - J * dS) / dS;

        % Prepare for next iteration
        s = s_new;
        F = F_new;
    end

    % Final solution integration
    y0 = [alpha; s];
    [X, Y] = ode45(@(x, y) odefun(x, y), [a b], y0);

    % Plot
    figure;
    plot(X, Y(:,1), 'b-', 'LineWidth', 2);
    xlabel('x'); ylabel('y(x)');
    title('BVP Solution via Broyden + Shooting');
    grid on;

end

% ODE function: y'' = -x * sin(y)
function dydx = odefun(x, y)
    dydx = zeros(2,1);
    dydx(1) = y(2);            % y' = v
    dydx(2) = -x * sin(y(1));  % v' = -x * sin(y)
end
