%
%------- Basic system model
%
clear; close all; clc;
C=blkdiag(eye(3),zeros(4));
n=7; % number of states
m=3; % number of inputs
g=9.81;
% b=[b_gamma;b_va;b_phi]
b_gamma=1.5;
b_va=0.6;
b_phi=7;
b=[b_gamma;b_va;b_phi];
%
%------- Parameters --------
%
N=5;
h = 0.1;
ywp0 = [0;0;20];
x0=[ywp0;deg2rad(0);0;5;0]; % y = [pn pe h chi gamma Va phi]
ywp = [
    5 0 20;
    10 5 16;
    10 15 20;
    15 20 20;
    20 20 20;
];
tol = 0.5;

%
%----- Cost ------------
%

fun =@(y) CostFunction(y,ywp(1,:)',N,h,m,n);


%------- MPC algorithm ------

%
M=200; %time horizon
xt=ones(n*N+m*N,1);
zt=x0;
yvec=zt';
uvec=[];
options = optimoptions("fmincon","Display","none","Algorithm","sqp",...
   "EnableFeasibilityMode",true,"SubproblemAlgorithm","cg");
% nonlicon =@(y) dynConstraint(h,N,y,m,n,x0,g,b);
% Let y=[u0^T u1^T ... u_{N-1}^T x1^T  ...  xN^T]^T
% [y,fval]=fmincon(fun,xt,[],[],[],[],[],[],nonlicon,options);

lbw=kron(ones(N,1),eye(3))*[-deg2rad(30);5;-deg2rad(60)];
lbw=[lbw;-ones(n*N,1)*inf];
ubw=kron(ones(N,1),eye(3))*[deg2rad(20);15;deg2rad(60)];
ubw=[ubw;ones(n*N,1)*inf];
i=1;
wp=ywp(1,:)';
for flcnt1=1:M
    x0=zt;
    nonlicon =@(y) dynConstraint(h,N,y,m,n,x0,g,b);
    y=fmincon(fun,xt,[],[],[],[],lbw,ubw,nonlicon,options);
    ut=y(1:3);
    f_NL = NonlinearDyn(zt,ut,g,b)';
    zt=zt+h*f_NL;
    yvec=[yvec;zt'];
    uvec=[uvec;reshape(ut,[1,3])];
    flcnt1
    if norm(zt(1:3) - ywp(end,:)') < tol
        break;
    end
    if norm(zt(1:3) - wp) < tol
        i = i+1;
        wp = ywp(i,:)';
        fun =@(y) CostFunction(y,wp,N,h,m,n);
    end

end

M_total = flcnt1;
tvec=h*(1:1:M);
% subplot(3,1,1)  %For the other two sets of parameters you should change
                %the third index to 2 and 3, respectively.
%plot(tvec,yvec,'-',tvec,uvec,'--')
yvec = round(yvec,2);
figure
plot3(yvec(:,1),yvec(:,2),yvec(:,3),'LineWidth', 2)
hold on
loc = 1:length(ywp)-1;
plot3(ywp(loc,1), ywp(loc,2), ywp(loc,3), 'rx',...
    'MarkerSize', 10, 'LineWidth', 2);
scatter3(ywp0(1),ywp0(2),ywp0(3),'d','filled',...
    'MarkerFaceColor',"#EDB120",'MarkerEdgeColor',"#EDB120",...
    "LineWidth",2);
scatter3(ywp(end,1),ywp(end,2),ywp(end,3),'s','filled',...
    'MarkerFaceColor',"#A2142F",'MarkerEdgeColor',"#A2142F",...
    "LineWidth",2);
xlabel('x'); ylabel('y'); zlabel('h');
title(sprintf('UAV Trajectory from MPC method (Optimized T = %.2f s)', h*(M_total-1)));
legend('Trajectory', 'Waypoints','Start','Finish'); grid on;
grid on

% Plot the control
k = 0:M_total-1;              % Discrete time indices
T_all = k * h;            % Map discrete time to continuous time

figure
subplot(2,1,1);
stairs(T_all,uvec(:,2),'LineWidth',1)
ylabel('Control input V_a (m/s)')
title('Time history of V_a command')
grid on

subplot(2,1,2); 
stairs(T_all,rad2deg(uvec(:,1)),'-.','LineWidth',1.5)
hold on
stairs(T_all,rad2deg(uvec(:,3)),'--*','LineWidth',1.5)
xlabel('Time (s)')
title('Time history of \gamma_c and \phi_c')
ylabel('Control Inputs (rad)')
grid on
legend('\gamma_c','\phi_c')

% Plot the velocity, gamma, phi, psi
% y = [pn pe h chi gamma Va phi]
kk=0:M_total;
T_all2 = kk' * h;  
figure
subplot(4,1,1)
plot(T_all2,yvec(:,6),'LineWidth',1.5)
ylabel('Velocity (m/s)')
title('Time history of the velocity, \gamma, \psi, and \phi')
grid on

subplot(4,1,2)
plot(T_all2,rad2deg(yvec(:,5)),'LineWidth',1.5)
ylabel('\gamma (deg)')
grid on

subplot(4,1,3)
plot(T_all2,rad2deg(yvec(:,4)),'LineWidth',1.5)
ylabel('\psi (deg)')
grid on

subplot(4,1,4)
plot(T_all2,rad2deg(yvec(:,7)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\phi (deg)')
grid on

%% Dynamics constraint

% y1 = pa; y2 = pe; y3 = h; y4 = psi; y5 = gamma; y6 = Va; y7 = phi;
% y8=u1=gamma_c; y9=u2=Va_c; y10=u3=phi_c
function [c,ceq] = dynConstraint(h,N,y,m,n,x0,g,b)
    % Let y=[u0^T u1^T ... u_{N-1}^T x1^T  ...  xN^T]^T
    imax = N-1;
    j=4;k=1;l=k+7;
    u_parse = y(1:m*N);
    y_parse=y(m*N+1:(n+m)*N);
    f_NL0 = NonlinearDyn(x0,u_parse(1:3),g,b)';
    ceq=y_parse(1:7)-x0-h*f_NL0;
    for i = 1:imax
        u_new=u_parse(j:j+2);
        y_new=y_parse(k:k+6);
        f_NL = NonlinearDyn(y_new,u_new,g,b)';
        ceq_iter=y_parse(l:l+6)-y_new-h*f_NL;
        % ceq_iter=ceq_iter+h*f_NL;
        ceq=[ceq;ceq_iter];
        j=j+3;
        k=k+7;
        l=l+7;
    end
    c=[];
end

% Cost function
function fun = CostFunction(y,ywp,N,h,m,n)
    fun = norm(ywp(1)-y((n+m)*N-6))^2+...
        norm(ywp(2)-y((n+m)*N-5))^2+...
        norm(ywp(3)-y((n+m)*N-4))^2;
    % j = 1;
    % k = m*N+1;
    for i= 1:m*N
        u_cost=h*y(i)^2+h*y(i+1)^2+h*y(i+2)^2;
        % u_cost=h*y(i)^2;
        fun = fun+u_cost; %+x_cost;
        % j = j+3;
        % k = k+7;
    end
end

%% Function to calculate nonlinear part of the dynamics
function f_NL = NonlinearDyn(y,u,g,b)
    % y = [pn pe h chi gamma Va phi]
    f_NL(1)=y(6)*cos(y(4))*cos(y(5));
    f_NL(2)=y(6)*sin(y(4))*cos(y(5));
    f_NL(3)=y(6)*sin(y(5));
    f_NL(4)=g/y(6)*tan(y(7));
    f_NL(5)=b(1)*(u(1)-y(5));
    f_NL(6)=b(2)*(u(2)-y(6));
    f_NL(7)=b(3)*(u(3)-y(7));
end