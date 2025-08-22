% Sample data
k = 0:10;              % Discrete time indices
u = sin(0.3*k);        % Discrete data
Ts = 0.1;              % Sampling time

t = k * Ts;            % Map discrete time to continuous time

stairs(t, u, 'LineWidth', 2);  % ZOH-style plot
xlabel('Time (s)');
ylabel('u(t)');
title('Discrete-Time Variable as Zero-Order Hold');
grid on;