%
%------- Basic system model
%
clear; close all; clc;
C=blkdiag(eye(3),zeros(4));
n=7; % number of states
m=3; % number of inputs
g=9.81;
% b=[b_gamma;b_va;b_phi]
b_gamma=0.5;
b_va=0.5;
b_phi=0.5;
b=[b_gamma;b_va;b_phi];
%
%------- Parameters --------
%

N=20;
h = 0.1;
x0=[0;0;20;0;0;5;0];
ywp = [10;0;20];

%
%----- Cost ------------
%
% norm(ywp,y(n*(N-1)+1:n*(N-1)+3))
fun =@(y) CostFunction(y,ywp,N,h,m,n);

%---------States and Inputs bound---------
% lb=[]


%------- Discretization ------

xt=ones(n*N+m*N,1);
options = optimoptions("fmincon","Algorithm","sqp",...
   "EnableFeasibilityMode",true,"SubproblemAlgorithm","cg");
nonlicon =@(y) dynConstraint(h,N,y,m,n,x0,g,b);

% Let y=[u0^T u1^T ... u_{N-1}^T x1^T  ...  xN^T]^T
[y,fval]=fmincon(fun,xt,[],[],[],[],[],[],nonlicon,options);

% Parsing the results into x_f, x_l, and u
% x = [pn pe h chi gamma Va phi]
y_u = y(1:m*N);
y_x = y(m*N+1:(m+n)*N);

% Input u1=gamma_c; u2=Va_c; u3=phi_c
u   = zeros(N,m);

% position in NED frames x_NED = [pn pe h]
x_NED = [x0(1) x0(2) x0(3);zeros(N,3)];

% Attitude angle (phi, gamma, chi) i.e. (roll, pitch, yaw)
chi = [x0(4);zeros(N,1)];
gamma = [x0(5);zeros(N,1)];
phi = [x0(7);zeros(N,1)];

% Aircraft velocity magnitude
Va = [x0(6);zeros(N,1)];

% Begin parsing using the convention:
% x1 = pa; x2 = pe; x3 = h; x4 = psi; x5 = gamma; x6 = Va; x7 = phi; 
j = 1;
k = 2;

% Input
for i=1:m:m*N-2
    u(j,:) = [rad2deg(y_u(i)) y_u(i+1) rad2deg(y_u(i+2))];
    j=j+1;
end
% States
for i=1:n:n*N-6
    x_NED(k,:)=[y_x(i) y_x(i+1) y_x(i+2)];
    chi(k)=rad2deg(y_x(i+3));
    gamma(k)=rad2deg(y_x(i+4));
    Va(k)=y_x(i+5);
    phi(k)=rad2deg(y_x(i+6));
    k = k+1;
end
r=round(x_NED(:,1:3),2);
plot3(r(:,1),r(:,2),r(:,3))
xlabel('x');
ylabel('y');
zlabel('z');





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
    % ceqN = [y((n+m)*N-6)-ywp(1);y((n+m)*N-5)-ywp(2);...
    %     y((n+m)*N-4)-ywp(3)];
    % ceq=[ceq;ceqN];

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