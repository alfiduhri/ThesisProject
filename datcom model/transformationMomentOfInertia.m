% Matrix of Inertia
I = [0.051 0.021 -0.001;0.021 0.031 0;-0.001 0 0.082];

% Rotation of coordinates for moment of inertia tensor
% The transformation for xb = -y, yb = -x, zb=-z
T = [0 -1 0;-1 0 0;0 0 -1];

Ib=transpose(T)*I*T; % Matrix of inertia with the center of global coord
                       % with the direction the same as body-fixed ref.


% Parallel Axis Theorem
% I' = I+M*[(R,R)]-2*M*[(R,C)]
% I is Icm or moment inertia with respect to center of mass

M = 0.9; % kg

C=[0;0;0];
R=[-0.21378;0;0];

Jb=Ib-M*symm(R,R)+2*M*symm(R,C);