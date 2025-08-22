waypoints = [
0	0	20;
5	0	20;
10	5	20;
10	15	20;
15	20	20;
20	20	20;
];
figure
plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3))
tspan = [0 100];
wp_ts = enumWP(waypoints,tspan);
figure
plot3(wp_ts(:,2),wp_ts(:,3),wp_ts(:,4))

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