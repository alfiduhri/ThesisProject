ywp0 = [0;0;20];
ywp = [
    5 0 20;
    10 5 16;
    10 15 20;
    15 20 20;
    20 20 20;
];
figure
plot3(X_opt(1,:), X_opt(2,:), X_opt(3,:),'LineWidth', 2)
hold on
plot3(yvec(:,1),yvec(:,2),yvec(:,3),'LineWidth', 2)
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
legend('Discretization','MPC', 'Waypoints','Start','Finish'); grid on;
grid on

% Plot the control
figure
subplot(3,1,1);
stairs(T_all_1,U_opt(2,:),'LineWidth',1)
hold on
stairs(T_all,uvec(:,2),'LineWidth',1)
ylabel('Control input V_a (m/s)')
title('Time history of the control inputs')
grid on
legend('Disc','MPC')

subplot(3,1,2); 
stairs(T_all_1,rad2deg(U_opt(1,:)),'-.','LineWidth',1.5)
hold on
stairs(T_all,rad2deg(uvec(:,1)),'--*','LineWidth',1.5)
xlabel('Time (s)')
ylabel('\gamma_c (deg)')
grid on


subplot(3,1,3);
stairs(T_all_1,rad2deg(U_opt(3,:)),'--*','LineWidth',1.5)
hold on
stairs(T_all,rad2deg(uvec(:,3)),'--*','LineWidth',1.5)
xlabel('Time (s)')
ylabel('\phi_c (deg)')
grid on


%Plot the velocity, roll angle, course angle, flight path angle
%State: [pn; pe; h; chi; gamma; Va; phi]  
figure
subplot(4,1,1)
plot(T_all2_1,X_opt(6,:),'LineWidth',1.5)
hold on
plot(T_all2,yvec(:,6),'LineWidth',1.5)
ylabel('Velocity (m/s)')
title('Time history of the velocity, \gamma, \psi, and \phi')
grid on
legend('Disc','MPC')

subplot(4,1,2)
plot(T_all2_1,rad2deg(X_opt(5,:)),'LineWidth',1.5)
hold on
plot(T_all2,rad2deg(yvec(:,5)),'LineWidth',1.5)
ylabel('\gamma (deg)')
grid on


subplot(4,1,3)
plot(T_all2_1,rad2deg(X_opt(4,:)),'LineWidth',1.5)
hold on
plot(T_all2,rad2deg(yvec(:,4)),'LineWidth',1.5)
ylabel('\psi (deg)')
grid on


subplot(4,1,4)
plot(T_all2_1,rad2deg(X_opt(7,:)),'LineWidth',1.5)
hold on
plot(T_all2,rad2deg(yvec(:,7)),'LineWidth',1.5)
xlabel('Time (s)')
ylabel('\phi (deg)')
grid on
