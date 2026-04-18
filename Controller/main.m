close all; clear; clc;
addpath 'C:\Users\srikr\Desktop\SPARX\Nav\Full State Estimation'

patch_size = 10;        
N_res = 200;            
sim_freq = 10;          
t_vec = 0:30:200;       

my_test = Testcase(patch_size, N_res, sim_freq, t_vec, 0, 0.055);
my_test.plot_scenario();

geom.r = 0.17/2;
geom.B = 0.2;
geom.L = 0.25;

dt = 0.01;           
mission_time = 0;   
time_compression = (14 * 24 * 3600) / 2000; 

u = [1.0; 1.0 * 0.75]; 

lander_pos = [my_test.traj.X(1); my_test.traj.Y(1); my_test.Z_traj(1)];
curr_state = [lander_pos; 0; 0; 0; 0; 0; 0];

scale = 0.8;
sun_scale = 6;

hold on;
hSun = quiver3(lander_pos(1), lander_pos(2), lander_pos(3), 0,0,0, 0, 'Color', [1 1 0], 'LineWidth', 2, 'AutoScale', 'off');
hSunPt = plot3(0,0,0, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8);

hFwd = quiver3(0,0,0,0,0,0, 0, 'r', 'LineWidth', 2, 'AutoScale', 'off'); 
hLft = quiver3(0,0,0,0,0,0, 0, 'g', 'LineWidth', 2, 'AutoScale', 'off'); 
hUp  = quiver3(0,0,0,0,0,0, 0, 'b', 'LineWidth', 2, 'AutoScale', 'off');
hRover = plot3(lander_pos(1), lander_pos(2), lander_pos(3), 'wo', 'MarkerSize', 6, 'MarkerFaceColor', 'w');

hRover_quiver = quiver3(lander_pos(1), lander_pos(2), lander_pos(3), ...
    curr_state(1), curr_state(2), curr_state(3), 'o', 'LineWidth', 2, 'AutoScale', 'off');
hRover_sun = quiver3(curr_state(1), curr_state(2), curr_state(3),0,0,0, 'o', 'LineWidth', 2,'AutoScale', 'off');
h_attitude = quiver3(curr_state(1), curr_state(2), curr_state(3),0,0,0, 'w', 'LineWidth', 2,'AutoScale', 'off');

i = 0;
while true
    i = i + 1;
    mission_time = mission_time + (dt * time_compression);
    days_elapsed = mission_time / (24 * 3600);
    
    u = u + randn(2,1);
    u = u / (1.01);

    curr_state = curr_state + dt * FullStateDynamics(curr_state, u, my_test, geom);
    p = curr_state(1:3);
    
    s_dir = LanderSunPointingVector(2831, days_elapsed);
    
    set(hSun, 'UData', s_dir(1)*sun_scale, 'VData', s_dir(2)*sun_scale, 'WData', s_dir(3)*sun_scale);
    set(hSunPt, 'XData', lander_pos(1)+s_dir(1)*sun_scale, 'YData', lander_pos(2)+s_dir(2)*sun_scale, 'ZData', lander_pos(3)+s_dir(3)*sun_scale);
    set(hRover, 'XData', p(1), 'YData', p(2), 'ZData', p(3));
    
    set(hRover_quiver, 'UData', p(1) - lander_pos(1), 'VData', p(2) - lander_pos(2), 'WData', p(3) - lander_pos(3));
    set(hRover_sun, ...
        'XData', p(1), 'YData', p(2), 'ZData', p(3),...
        'UData', s_dir(1) * sun_scale - (p(1) - lander_pos(1)), ...
        'VData', s_dir(2) * sun_scale - (p(2) - lander_pos(2)), ...
        'WData', s_dir(3)* sun_scale - (p(3) - lander_pos(3)));
        
    y = curr_state(7); pitch = curr_state(8); r = curr_state(9);
    R = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1] * ...
        [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)] * ...
        [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
    
    % --- TRIAD SETUP (FIXED) ---
    R_r_gravity = R' * [0; 0; -1];
    R_r_gravity = R_r_gravity / norm(R_r_gravity);
    
    R_r_sun = R' * s_dir;
    R_r_sun = R_r_sun / norm(R_r_sun);
    
    L_r_sun = s_dir / norm(s_dir);
    
    rot_matrix = TRIAD(L_r_sun, R_r_sun, R_r_gravity);
    
    R_attitude_rover = [1 ; 0; 0];
    
    % Apply scale so the white arrow is long enough to see over the rover
    L_attitude_rover = rot_matrix * R_attitude_rover * scale; 
    
    set(h_attitude, ...
        'XData', p(1), 'YData', p(2), 'ZData', p(3),...
        'UData', L_attitude_rover(1), ...
        'VData', L_attitude_rover(2), ...
        'WData', L_attitude_rover(3));
        
    % --- TRUE ATTITUDE PLOTTING ---
    f = R(:,1)*scale; l = R(:,2)*scale; u_vec = R(:,3)*scale;
    set(hFwd, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', f(1), 'VData', f(2), 'WData', f(3));
    set(hLft, 'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', l(1), 'VData', l(2), 'WData', l(3));
    set(hUp,  'XData', p(1), 'YData', p(2), 'ZData', p(3), 'UData', u_vec(1), 'VData', u_vec(2), 'WData', u_vec(3));
        
    drawnow limitrate;
    
    if any(p(1:2) > patch_size) || any(p(1:2) < 0)
        fprintf('Rover reached map boundary at day %.2f\n', days_elapsed);
        break;
    end
end