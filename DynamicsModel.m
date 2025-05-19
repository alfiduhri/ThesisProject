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
x0 = [0;0;20;chi_0;gamma_0;Va_0;phi_0];

tspan = [0 100];

[t,x]=ode45(@(t,x) dynGuidance(t,x,bg,bva,bphi,wn,we,wd,g,u), tspan, x0);

function xdot = dynGuidance(t,x,bg,bva,bphi,wn,we,wd,g,u)
    % Controller definition
    gamma_c = u(1);Va_c = u(2);phi_c = u(3);

    % State definition
    chi=x(4);gamma = x(5); Va=x(6); phi=x(7);
    
    % Equation to calculate Vg
    Wn3=cos(chi)*cos(gamma)*wn+sin(chi)*cos(gamma)*we-sin(gamma)*wd;
    Vw=sqrt(wn^2+we^2+wd^2);
    % For example, p = [3 2 -2] represents the polynomial 3x^2+2x−2.
    p=[1 -2*Wn3 Vw^2-Va^2];
    r=roots(p);
    real_roots = r(abs(imag(r)) < 1e-6);
    Vg=real_roots(real_roots > 0);
    % If Vg has 2 real roots then it will be difficult to determine

    % equation to calculate gamma_a
    gamma_a=asin((Vg*sin(gamma)+wd)/Va);
    % equation to calculate psi
    Wn2 = -wn*sin(chi)+we*cos(chi);
    psi=chi-asin(1/(Va*cos(gamma_a))*Wn2);

    % Dynamic equations
    pn_dot = Va*cos(psi)*cos(gamma_a)+wn;
    pe_dot = Va*sin(psi)*cos(gamma_a)+we;
    h_dot = sin(gamma_a)-wd;
    chi_dot = g/Vg*tan(phi)*cos(chi-psi);
    gamma_dot = bg*(gamma_c-gamma);
    Va_dot = bva*(Va_c-Va);
    phi_dot = bphi*(phi_c-phi);

    

    xdot=[pn_dot;pe_dot;h_dot;chi_dot;gamma_dot;Va_dot;phi_dot];

end
