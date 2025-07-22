% Model for Guidance
% The states are X^T = [pn pe h chi gamma Va phi]
% The controller are U^T = [gamma_c Va_c phi_c]
% bg, bva, bphi are positive coefficients related to the implementation
% of low-level autopilot loops

g=9.81; %m/s^2
bg = 0.08;
bva = 1;
bphi = 1;
wn=0;we=0;wd=0;
u = [deg2rad(0);0;deg2rad(0)];
chi_0 = deg2rad(30);
gamma_0 = 0;
Va_0 = 5;
phi_0 = 0;
tspan = [0 20];

% List of waypoints
waypoints = [
0	0	20;
5	0	20;
10	5	20;
10	15	20;
15	20	20;
20	20	20;
];
% Divide the waypoints evenly
wp_ts = enumWP(waypoints,tspan);

% 1. Make an initial guess of lambda_0
lambda_0 = ones(7,1);
x0 = [0;0;20;chi_0;gamma_0;Va_0;phi_0];

% 2. Integrate the system (x_dot and lambda_dot)
[~,lam]=ode45(@(t,lam) adjointSys(t,lam,bg,bva,bphi,g,wp_ts),tspan,lambda_0);
u = Controller(x,bg,bva,bphi);
[t,x]=ode45(@(t,x) tpbvpSys(t,x,bg,bva,bphi,g,u), tspan, x0);

% 3. Compute mu(tf) = mu(x(tf),lambda(tf))

% 4. Update lambda_0 = lambda_0 + d(lambda_0)
% 5. Repeat steps 2-4 until |mu(tf)| becomes sufficiently small

function lam_dot = adjointSys(t,x,bg,bva,bphi,g,wp_ts)
    % Interpolate waypoints at time t
    pn_wp = interp1(wp_ts(:,1),wp_ts(:,2),t);
    pe_wp = interp1(wp_ts(:,1),wp_ts(:,3),t);
    h_wp = interp1(wp_ts(:,1),wp_ts(:,4),t);
    % Adjoint function
    lam1 = x(1); lam2 = x(2); lam3 = x(3); lam4 = x(4); lam5 = x(5);
    lam6 = x(6); lam7 = x(7);
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
end

function xdot = tpbvpSys(t,x,bg,bva,bphi,g,u)
    % Controller definition
    % u1 = gamma_c; u2 = Va_c; u3 = phi_c;

    % State equations
    x1_dot = x(4)*cos(x(5))*cos(x(6));
    x2_dot = x(4)*sin(x(5))*cos(x(6));
    x3_dot = x(4)*sin(x(6));
    x4_dot = bva*(u(2)-x(4));
    x5_dot = g/x(4)*tan(x(7));
    x6_dot = bg*(u(1)-x(6));
    x7_dot = bphi*(u(3)-x(7));

    xdot=[x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot];
    % x_augmented=[xdot;lam1_dot;lam2_dot;lam3_dot;lam4_dot;...
    %     lam5_dot;lam6_dot;lam7_dot];
end

function u = Controller(x,bg,bva,bphi)
    % Take the corresponding lambda from x
    lam4 = x(4); lam6 = x(6); lam7 = x(7);
    % Optimal control input
    % u1 = gamma_c
    if -bg*lam6/2 > deg2rad(20)
        gamma_c = deg2rad(20);
    elseif -bg*lam6/2 < -deg2rad(30)
        gamma_c = -deg2rad(30);
    else
        gamma_c = -lam6*bg/2;
    end

    % u2 = Va_c
    Va_c = -lam4*bva/2;

    % u3 = phi_c
    if -bphi*lam7/2 > deg2rad(60)
        phi_c = deg2rad(60);
    elseif -bphi*lam7/2 < -deg2rad(60)
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
