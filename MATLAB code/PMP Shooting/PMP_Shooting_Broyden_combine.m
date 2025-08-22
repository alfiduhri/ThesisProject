% Model for Guidance
% The states are X^T = [pn pe h chi gamma Va phi]
% The controller are U^T = [gamma_c Va_c phi_c]
% bg, bva, bphi are positive coefficients related to the implementation
% of low-level autopilot loops

g=9.81; %m/s^2
bg = 1.5;
bva = 0.6;
bphi = 7;
wn=0;we=0;wd=0;
chi_0 = deg2rad(0);
gamma_0 = 0;
Va_0 = 5;
phi_0 = 0;
tspan = [0 8];

% List of waypoints
waypoints = [
5	0	20;
10	5	20;
10	15	20;
15	20	20;
20	20	20;
];
% Divide the waypoints evenly
wp_ts = enumWP(waypoints,tspan);

% 1. Make an initial guess of lambda_0
lambda0 = [0.2*ones(3,1);zeros(4,1)];
x0 = [0;0;20;chi_0;gamma_0;Va_0;phi_0];

% 2. Integrate the system (x_dot and lambda_dot)
% [t,x]=ode45(@(t,x) tpbvpSys(t,x,bg,bva,bphi,g,wp_ts), tspan, [x0;lambda_0]);
% plot(t,x(:,1))
% hold on
% plot(t,x(:,2))
% plot(t,x(:,3))

% 3. Compute mu(tf) = mu(x(tf),lambda(tf))
% Want to make mu_tf = 0 or close to 0
mu_tf = shooting_residual(x0,lambda0,waypoints,tspan,bg,bva,bphi,g,wp_ts);

% 4. Update lambda_0 = lambda_0 + d(lambda_0)
% 5. Repeat steps 2-4 until |mu(tf)| becomes sufficiently small
% Initial Jacobian approximation using finite difference
h = 1e-6;
J = finite_jacobian(lambda0,h,x0,waypoints,tspan,bg,bva,bphi,g,wp_ts);

tol = 1e-6;
max_iter = 30;

for k = 1:max_iter
    % Solve B * delta = -r
    delta_lambda = -pinv(J) \ mu_tf;
    lambda_new = lambda0 + delta_lambda;
        
    mu_new = shooting_residual(x0,lambda_new,waypoints,tspan,bg,bva,bphi,g,wp_ts);

    % Broyden update
    y = mu_new - mu_tf;
    s = delta_lambda;
    J = J + ((y - J * s) * s') / (s' * s);

    fprintf('Iter %d: Residual norm = %.4e\n', k, norm(mu_new));

    if norm(s) < tol
        lambda0 = lambda_new;
        break;
    end

    % Update
    lambda0 = lambda_new;
    mu_tf = mu_new;
end

% Final integration with updated lambda0
[t,x]=ode45(@(t,x) tpbvpSys(t,x,bg,bva,bphi,g,wp_ts), tspan, [x0;lambda0]);
plot3(x(:,1),x(:,2),x(:,3))
xlabel('x')
ylabel('y')
zlabel('h')

function xdot_aug = tpbvpSys(t,x,bg,bva,bphi,g,wp_ts)
    % Controller definition
    % u1 = gamma_c; u2 = Va_c; u3 = phi_c;
    
    % Controller calculation
    u = Controller(x,bg,bva,bphi);

    % Interpolate waypoints at time t
    pn_wp = interp1(wp_ts(:,1),wp_ts(:,2),t);
    pe_wp = interp1(wp_ts(:,1),wp_ts(:,3),t);
    h_wp = interp1(wp_ts(:,1),wp_ts(:,4),t);

    % Adjoint function
    lam1 = x(8); lam2 = x(9); lam3 = x(10); lam4 = x(11); lam5 = x(12);
    lam6 = x(13); lam7 = x(14);
    lam1_dot=2*(pn_wp-x(1));
    lam2_dot=2*(pe_wp-x(2));
    lam3_dot=2*(h_wp-x(3));
    lam4_dot=-lam1*cos(x(5))*cos(x(6))-lam2*sin(x(5))*cos(x(6))...
        -lam3*sin(x(6))+lam4*bva+lam5*g/(x(4))^2*tan(x(7));
    lam5_dot=(lam1*sin(x(5))-lam2*cos(x(5)))*x(4)*cos(x(6));
    lam6_dot=x(4)*sin(x(6))*(lam1*cos(x(5))+lam2*sin(x(5)))-...
        lam3*x(4)*cos(x(6))+lam6*bg;
    lam7_dot=-lam5*g/x(4)*(sec(x(7)))^2+lam7*bphi;

    lam_dot = [lam1_dot;lam2_dot;lam3_dot;lam4_dot;...
        lam5_dot;lam6_dot;lam7_dot];

    % State equations
    x1_dot = x(4)*cos(x(5))*cos(x(6));
    x2_dot = x(4)*sin(x(5))*cos(x(6));
    x3_dot = x(4)*sin(x(6));
    x4_dot = bva*(u(2)-x(4));
    x5_dot = g/x(4)*tan(x(7));
    x6_dot = bg*(u(1)-x(6));
    x7_dot = bphi*(u(3)-x(7));

    xdot=[x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot];

    xdot_aug = [xdot;lam_dot];
end


function u = Controller(x,bg,bva,bphi)
    % Take the corresponding lambda from x
    % Lam1 = x8, lam2 = x9, etc.
    lam4 = x(11); lam6 = x(13); lam7 = x(14);
    % Optimal control input
    % u1 = gamma_c
    if -bg*lam6/2 >= deg2rad(20)
        gamma_c = deg2rad(20);
    elseif -bg*lam6/2 <= -deg2rad(30)
        gamma_c = -deg2rad(30);
    else
        gamma_c = -lam6*bg/2;
    end

    % u2 = Va_c
    if -bva*lam4/2 >= 15
        Va_c = 15;
    elseif -bva*lam4/2 <= 5
        Va_c = 5;
    else
        Va_c = -lam4*bva/2;
    end

    % u3 = phi_c
    if -bphi*lam7/2 >= deg2rad(60)
        phi_c = deg2rad(60);
    elseif -bphi*lam7/2 <= -deg2rad(60)
        phi_c = -deg2rad(60);
    else
        phi_c = -lam7*bphi/2;
    end

    % Combining the controller
    u = [gamma_c;Va_c;phi_c];
end

function wp_ts = enumWP(wp,tspan)
    % Number of points
    n = 1000;

    % Define the waypoints
    l_wp = length(wp);
    n_iter = n/(l_wp-1);
    pn_w = [];
    pe_w = [];
    h_w = [];
    for i = 1:l_wp-1
        pn_w = [pn_w linspace(wp(i,1),wp(i+1,1),n_iter)];
        pe_w = [pe_w linspace(wp(i,2),wp(i+1,2),n_iter)];
        h_w = [h_w linspace(wp(i,3),wp(i+1,3),n_iter)];
    end

    % Time of the waypoints
    % the format is tspan = [tspan(1) tspan(2)]
    t_wp = linspace(tspan(1),tspan(end),n);

    wp_ts = [t_wp' pn_w' pe_w' h_w'];
end

function r = shooting_residual(x0,lambda0,wp,tspan,bg,bva,bphi,g,wp_ts)
    y0 = [x0; lambda0];
    [~, Y] = ode45(@(t, y) tpbvpSys(t,y,bg,bva,bphi,g,wp_ts), tspan, y0);
    x_final = [Y(end, 1:3) Y(end,11:14)]';
    r = x_final - [wp(end,1); wp(end,2); wp(end,3);zeros(4,1)];
end

function J = finite_jacobian(lambda0,h,x0,wp,tspan,bg,bva,bphi,g,wp_ts)
    n = length(lambda0);
    J = zeros(7, n);
    for i = 1:n
        d = zeros(n,1); d(i) = h;
        r1 = shooting_residual(x0,lambda0 + d,wp,tspan,bg,bva,bphi,g,wp_ts);
        r0 = shooting_residual(x0,lambda0,wp,tspan,bg,bva,bphi,g,wp_ts);
        J(:,i) = (r1 - r0) / h;
    end
end