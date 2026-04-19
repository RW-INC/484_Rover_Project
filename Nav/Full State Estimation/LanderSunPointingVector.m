function pointing_vector = LanderSunPointingVector(t_lunar_day, t_relative)
    % --- 1. CONSTANTS ---
    tau = 1.54;
    Y_period = 365.25;
    n = 172;
    t0 = 0;
    L = deg2rad(-90); % South Pole
    T_lunar = 708;
    omega = 360 / T_lunar;
    t_lunar_offset = (t_lunar_day-2831) * 24;
    beta = zeros(1, length(t_relative));
    phi_s = zeros(1, length(t_relative));
    
    % --- 2. CALCULATION LOOP ---
    for i = 1:length(t_relative)
        ti = t_relative(i);
        t_abs = ti + t_lunar_offset;
        
        % Solar declination (must be radians for trig below)
        delta = deg2rad(tau * sin(deg2rad(360 / Y_period) * (n + t_abs/24 - t0)));
        
        % Hour angle
        H = deg2rad(omega * (T_lunar / 2 - t_abs));
        
        % Solar elevation
        beta(i) = asin(cos(L) * cos(delta) * cos(H) + sin(L) * sin(delta));
        
        % Solar Azimuth 
        phi_s(i) = atan2(cos(delta) * sin(H), ...
            sin(L) * cos(delta) * cos(H) - cos(L) * sin(delta) ...
        );
    end
    
    % --- 3. GENERATE VECTORS ---
    % X = East, Y = North, Z = Up
    pointing_vector = [
        cos(beta) .* sin(phi_s); 
        cos(beta) .* cos(phi_s); 
        sin(beta)
    ];
end