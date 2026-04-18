classdef Trajectory
    properties
        X, Y, Theta
        X_dot, Y_dot, Theta_dot
        t_master
        t_knots, X_pts, Y_pts
    end
    
    methods
        function obj = Trajectory(sim_freq, t_vec, x0, y0, min_v, max_v)
            dt = 1/sim_freq;
            obj.t_master = t_vec(1):dt:t_vec(end);
            
            % Generate knots
            [obj.t_knots, obj.X_pts, obj.Y_pts] = obj.generate_testcases(t_vec, x0, y0, min_v, max_v);
            
            % Create Spline
            obj.X = spline(obj.t_knots, obj.X_pts, obj.t_master);
            obj.Y = spline(obj.t_knots, obj.Y_pts, obj.t_master);
            obj.X_dot = gradient(obj.X) / dt;
            obj.Y_dot = gradient(obj.Y) / dt;
            
            % Heading logic
            obj.Theta = unwrap(atan2(obj.Y_dot, obj.X_dot));
            obj.Theta_dot = gradient(obj.Theta) / dt;
        end
        
        function [t_k, x_p, y_p] = generate_testcases(~, t_vec, x0, y0, MIN_V, MAX_V)
            n = length(t_vec);
            t_k = linspace(t_vec(1), t_vec(end), n);
            x_p = zeros(1,n); y_p = zeros(1,n);
            x_p(1) = x0; y_p(1) = y0;
            for i = 2:n
                dt_k = t_k(i) - t_k(i-1);
                v = MIN_V + (MAX_V - MIN_V) * rand();
                r = v * dt_k;
                ang = (1.2 * pi) * rand();
                x_p(i) = x_p(i-1) + r * cos(ang);
                y_p(i) = y_p(i-1) + r * sin(ang);
            end
        end
    end
end