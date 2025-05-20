subplot(2,1,1);
stairs(T_all,[U_opt(2,:) nan],'LineWidth',1)
ylabel('Control input V_a (m/s)')
title('Time history of V_a command')
grid on

subplot(2,1,2); 
stairs(T_all,[U_opt(1,:) nan],'-.','LineWidth',1)
hold on
stairs(T_all,[U_opt(3,:) nan],'--*','LineWidth',1)
xlabel('Time (s)')
title('Time history of \gamma_c and \phi_c')
ylabel('Control Inputs (rad)')
grid on
legend('\gamma_c','\phi_c')