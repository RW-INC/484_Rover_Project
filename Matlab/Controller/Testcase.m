classdef Testcase < handle
    properties
        patch_size, N_res
        Z_map
        dx, 
        vec,
        xmin, xmax
        traj 
        Z_traj 
    end
    
    methods
        function obj = Testcase(~, N_res, sim_freq, t_vec, min_v, max_v)
            obj.N_res = N_res;
            obj.traj = Trajectory(sim_freq, t_vec, 0, 0, min_v, max_v);

            % patch size is max dist from origin
            obj.patch_size = 2.0 * sqrt(max(abs(obj.traj.X))^2 + max(abs(obj.traj.Y))^2);
            obj.generate_terrain();

            obj.xmin = obj.vec(1);
            obj.xmax = obj.vec(end);
        end
        
        function generate_terrain(obj)
            wobble_amp  = obj.patch_size * 0.15;
            terrain_amp = obj.patch_size * 0.03;
            num_craters = 80;
            
            obj.vec = linspace(-obj.patch_size, obj.patch_size, obj.N_res);
            [X_map, Y_map] = meshgrid(obj.vec, obj.vec);
            obj.Z_map = zeros(obj.N_res, obj.N_res);
            
            wobble_nodes = 9;
            node_vec = linspace(-obj.patch_size, obj.patch_size, wobble_nodes);
            [node_X, node_Y] = meshgrid(node_vec, node_vec);
            
            raw_wobble = randn(wobble_nodes, wobble_nodes);
            wobble_terrain = interp2(node_X, node_Y, raw_wobble, X_map, Y_map, 'spline');

            wobble_terrain = (wobble_terrain / max(abs(wobble_terrain(:)))) * wobble_amp;
            obj.Z_map = obj.Z_map + wobble_terrain;
            
            for i = 1:5
                freq = (i * 0.1) / obj.patch_size;
                amp  = terrain_amp / i;
                obj.Z_map = obj.Z_map + ...
                    amp * sin(2*pi * freq * X_map + rand*2*pi) .* ...
                          cos(2*pi * freq * Y_map + rand*2*pi);
            end
            
            for c = 1:num_craters
                c_x = rand * obj.patch_size;
                c_y = rand * obj.patch_size;
                r = rand * (obj.patch_size * 0.05) + (obj.patch_size * 0.01); 
                d = rand * (obj.patch_size * 0.01) + (obj.patch_size * 0.002); 
                dist_sq = (X_map - c_x).^2 + (Y_map - c_y).^2;
                obj.Z_map = obj.Z_map - d * exp(-dist_sq / (2 * (r * 0.7)^2)) ...
                                     + (d * 0.25) * exp(-dist_sq / (2 * r^2));
            end
            
            max_slope_deg = 55; 
            
            obj.dx = (obj.vec(end) - obj.vec(1)) / (obj.N_res - 1);
            [dzdx, dzdy] = gradient(obj.Z_map, obj.dx, obj.dx);
            max_grad = max(sqrt(dzdx(:).^2 + dzdy(:).^2));
            target_grad = tan(deg2rad(max_slope_deg));

            if max_grad > target_grad
                obj.Z_map = obj.Z_map * (target_grad / max_grad);
            end

            hover_height = obj.patch_size * 0.0005;
            obj.Z_traj = interp2(X_map, Y_map, obj.Z_map, ...
                                 obj.traj.X, obj.traj.Y, 'linear') + hover_height;
        end
        
       function plot_scenario(obj, ax)
            if nargin < 2
                figure('Position', [100 100 900 700], 'Color', 'k');
                ax = gca;
            end
        
            axes(ax);
            hold on; axis equal;
        
            margin = obj.patch_size * 0.5;
            x_min = min(obj.traj.X) - margin;
            x_max = max(obj.traj.X) + margin;
            
            y_min = min(obj.traj.Y) - margin;
            y_max = max(obj.traj.Y) + margin;
        
            side = max(x_max - x_min, y_max - y_min);
            cx = 0.5 * (x_min + x_max);
            cy = 0.5 * (y_min + y_max);
            xlim(ax, [cx - side/2, cx + side/2]);
            ylim(ax, [cy - side/2, cy + side/2]);
            axis off;
        
            surf(ax, obj.vec, obj.vec, obj.Z_map, ...
                 'FaceAlpha', 0.9, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
                 'DisplayName','Lunar Surface');
            plot3(ax, obj.traj.X, obj.traj.Y, obj.Z_traj, 'r-', 'LineWidth', 2.5, ...
                'DisplayName', 'Nominal Trajectory');
            plot3(ax, obj.traj.X(1),   obj.traj.Y(1),   obj.Z_traj(1),   'go', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start Point');
            plot3(ax, obj.traj.X(end), obj.traj.Y(end), obj.Z_traj(end), 'ro', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End Point');
        
            colormap(ax, gray(256)); material dull;
            light('Parent', ax, 'Position', [obj.patch_size, 0, obj.patch_size], 'Style', 'local');
            view(ax, -35, 45);
            title(ax, 'Centered Trajectory', 'Color', 'w');
        end
    end
end