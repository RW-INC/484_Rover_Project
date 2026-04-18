classdef Testcase < handle
    properties
        patch_size, N_res
        X_map, Y_map, Z_map
        traj 
        Z_traj 
    end
    
    methods
        function obj = Testcase(patch_size, N_res, sim_freq, t_vec, min_v, max_v)
            obj.patch_size = patch_size;
            obj.N_res = N_res;
            
            obj.generate_terrain();
            
            cx = obj.patch_size / 2;
            cy = obj.patch_size / 2;
            obj.traj = Trajectory(sim_freq, t_vec, cx, cy, min_v, max_v);
            
            hover_height = obj.patch_size * 0.005;
            obj.Z_traj = interp2(obj.X_map, obj.Y_map, obj.Z_map, ...
                                 obj.traj.X, obj.traj.Y, 'linear') + hover_height;
        end
        
        function generate_terrain(obj)
            wobble_amp  = obj.patch_size * 0.015;
            terrain_amp = obj.patch_size * 0.003;
            num_craters = 80;
            
            vec = linspace(0, obj.patch_size, obj.N_res);
            [obj.X_map, obj.Y_map] = meshgrid(vec, vec);
            obj.Z_map = zeros(obj.N_res, obj.N_res);
            
            wobble_nodes = 8;
            node_vec = linspace(0, obj.patch_size, wobble_nodes);
            [node_X, node_Y] = meshgrid(node_vec, node_vec);
            raw_wobble = randn(wobble_nodes, wobble_nodes);
            wobble_terrain = interp2(node_X, node_Y, raw_wobble, obj.X_map, obj.Y_map, 'spline');
            wobble_terrain = (wobble_terrain / max(abs(wobble_terrain(:)))) * wobble_amp;
            obj.Z_map = obj.Z_map + wobble_terrain;
            
            for i = 1:3
                freq = (i * 5) / obj.patch_size;
                amp  = terrain_amp / i;
                obj.Z_map = obj.Z_map + ...
                    amp * sin(2*pi * freq * obj.X_map + rand*2*pi) .* ...
                          cos(2*pi * freq * obj.Y_map + rand*2*pi);
            end
            
            for c = 1:num_craters
                c_x = rand * obj.patch_size;
                c_y = rand * obj.patch_size;
                r = rand * (obj.patch_size * 0.05) + (obj.patch_size * 0.01); 
                d = rand * (obj.patch_size * 0.01) + (obj.patch_size * 0.002); 
                dist_sq = (obj.X_map - c_x).^2 + (obj.Y_map - c_y).^2;
                obj.Z_map = obj.Z_map - d * exp(-dist_sq / (2 * (r * 0.7)^2)) ...
                                     + (d * 0.25) * exp(-dist_sq / (2 * r^2));
            end
            
            max_slope_deg = 11; 
            dx = obj.patch_size / (obj.N_res - 1);
            [dzdx, dzdy] = gradient(obj.Z_map, dx, dx);
            max_grad = max(sqrt(dzdx(:).^2 + dzdy(:).^2));
            target_grad = tan(deg2rad(max_slope_deg));
            if max_grad > target_grad
                obj.Z_map = obj.Z_map * (target_grad / max_grad);
            end
        end
        
       function plot_scenario(obj, ax)
            if nargin < 2
                figure('Position', [100 100 900 700], 'Color', 'k');
                ax = gca;
            end
        
            axes(ax);
            hold on; axis equal;
        
            margin = obj.patch_size * 0.5;
            xmin = min(obj.traj.X) - margin;
            xmax = max(obj.traj.X) + margin;
            ymin = min(obj.traj.Y) - margin;
            ymax = max(obj.traj.Y) + margin;
        
            side = max(xmax - xmin, ymax - ymin);
            cx = 0.5 * (xmin + xmax);
            cy = 0.5 * (ymin + ymax);
            xlim(ax, [cx - side/2, cx + side/2]);
            ylim(ax, [cy - side/2, cy + side/2]);
            axis off;
        
            surf(ax, obj.X_map, obj.Y_map, obj.Z_map, ...
                 'FaceAlpha', 0.9, 'EdgeColor', 'none', 'FaceLighting', 'gouraud');
            plot3(ax, obj.traj.X, obj.traj.Y, obj.Z_traj, 'r-', 'LineWidth', 2.5);
        
            plot3(ax, obj.traj.X(1),   obj.traj.Y(1),   obj.Z_traj(1),   'go', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'g');
            plot3(ax, obj.traj.X(end), obj.traj.Y(end), obj.Z_traj(end), 'ro', ...
                  'MarkerSize', 10, 'MarkerFaceColor', 'r');
        
            colormap(ax, gray(256)); material dull;
            light('Parent', ax, 'Position', [obj.patch_size, 0, obj.patch_size], 'Style', 'local');
            view(ax, -35, 45);
            title(ax, 'Centered Trajectory', 'Color', 'w');
        end
    end
end