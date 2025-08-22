%
%------- Basic system model
%
clear;
T=0.1;
Phi=[1 T;0.5*T 1];
Gam=[T^2/2;T];
C=[1 0];
n=size(Phi,1);
m=size(Gam,2);
%
%------- Parameters --------
%
q=5;
r=1;
N=5;
z0=[0.5;1];
%
%------- Define matrices for the QP --------
%
Hz1 = kron(eye(N-1),2*q*(C'*C));
Hz2 = 2*q*eye(2);
Hu  = kron(eye(N),2*r);
H   = blkdiag(Hz1,Hz2,Hu);
f = [];

%
%----- For problem 2 with inequalities ---------
%
% A=[];
% b=[];
A1 = kron(eye(N),zeros(2));
A2 = kron(eye(N),1);
A3 = [zeros(N,length(A1)) kron(eye(N),-1)];
A = [blkdiag(A1,A2);A3];
b=[zeros(2*N,1);ones(2*N,1)];

%
%----- Cost ------------
%
Az1 = kron(eye(N),eye(2));
Az2 = [zeros(2,2*N);kron(eye(N-1),-Phi) zeros(2*N-2,2)];
Az  = Az1+Az2;
Au  = kron(eye(N),-Gam);
Aeq = [Az Au];
beq = [Phi*z0;zeros(2*N-2,1)];
AA = [Phi;zeros(2*N-2,2)]; % AA is defined as in the description below
                       % The change in beq will only happen for
                       % the first and second row with the rate Phi
                          

%

%------- MPC algorithm ------

%
M=100; %time horizon
zt=z0;
yvec=[];
uvec=[];
options=optimset('largescale','off');
for flcnt1=1:M
    beq=AA*zt;  % The matrix AA defines how the last measured state
                % determines the right hand side in the equality constraint.
    x=quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
    ut=x(n*N+1);
    zt=Phi*zt+Gam*ut;
    yvec=[yvec;C*zt];
    uvec=[uvec;ut];
end
tvec=T*(1:1:M);
%subplot(3,1,3)  %For the other two sets of parameters you should change
                %the third index to 2 and 3, respectively.
plot(tvec,yvec,'-',tvec,uvec,'--')
grid