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
N=10;
h = 0.1;
x0=[0;0;10;0;0;5;0];
ywp = [20;15;15];

%
%----- Cost ------------
%

fun =@(y) CostFunction(y,ywp,N,h,m,n);


%------- MPC algorithm ------

%
M=150; %time horizon
xt=ones(n*N+m*N,1);
zt=x0;
yvec=[];
uvec=[];
options = optimoptions("fmincon","Algorithm","sqp",...
   "EnableFeasibilityMode",true,"SubproblemAlgorithm","cg");
% nonlicon =@(y) dynConstraint(h,N,y,m,n,x0,g,b);
% Let y=[u0^T u1^T ... u_{N-1}^T x1^T  ...  xN^T]^T
% [y,fval]=fmincon(fun,xt,[],[],[],[],[],[],nonlicon,options);

for flcnt1=1:M
    x0=zt;
    nonlicon =@(y) dynConstraint(h,N,y,m,n,x0,g,b);
    y=fmincon(fun,xt,[],[],[],[],[],[],nonlicon,options);
    ut=y(1:3);
    f_NL = NonlinearDyn(zt,ut,g,b)';
    zt=zt+h*f_NL;
    yvec=[yvec;zt(1:3)'];
    uvec=[uvec;reshape(ut,[1,3])];
end
tvec=h*(1:1:M);
% subplot(3,1,1)  %For the other two sets of parameters you should change
                %the third index to 2 and 3, respectively.
%plot(tvec,yvec,'-',tvec,uvec,'--')
plot3(yvec(:,1),yvec(:,2),yvec(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
grid

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