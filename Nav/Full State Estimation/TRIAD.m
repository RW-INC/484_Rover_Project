function rotation_rover_to_lander = TRIAD(L_r_sun, R_r_sun, R_r_gravity)
    % The TRIAD fuction builds two rotation matrices for the rover and the
    % lunar lander in the LVLH frame. 
    
    % Find the LVLH matrix in Rover Frame. 
    R_t1 = R_r_gravity;
    R_t2 = cross(R_t1, R_r_sun); R_t2 = R_t2 / norm(R_t2);
    R_t3 = cross(R_t1, R_t2);

    % Find the LVLH matrix in Lunar Frame.
    L_t1 = [0 ; 0 ; -1]; % assume Z-axis points through the lander
    L_t2 = cross(L_t1, L_r_sun); L_t2 = L_t2 / norm(L_t2);
    L_t3 = cross(L_t1, L_t2);

    M_rover = [R_t1 R_t2 R_t3];
    M_lunar = [L_t1 L_t2 L_t3];

    rotation_rover_to_lander = M_lunar * M_rover';
end