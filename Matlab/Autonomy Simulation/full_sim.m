%% Clear workspace
clear all;
clc;
close all;

%% Setup controller parameters
K = [0.02 ; 0.02 ; 1.0];
eps_v = [5; 5; 5];

%% Setup rover geometry 
geom.r = 0.17/2;
geom.B = 0.2;
geom.L = 0.25;
max_w = 1.0;

%% Setup simulation parameters
f_physics = 1000;
f_estimation = 100;
f_control = 20;
map_resolution = 1000;


%%
% what is our process? 
%   Better if user supplies everything we just run the entire code. 
%   User inputs a function and function name => to get expected mex file 
%   We run codegen on the Controller + Simulation 
% ----   
%   We take the entire thing and package it into our simulation environment
%   and propagate it in realtime. 






% % codegen dynamics function (arbitrarily)
% arg_types = {coder.typeof(0, [9, 1]), ...
%     coder.typeof(0, [4, 1]), ...
%     coder.typeof(0), ...
%     coder.typeof(0), ...
%     coder.typeof(0), ...
%     coder.typeof(0), ...
%     coder.typeof(zeros(map_resolution, map_resolution)), ...
%     coder.typeof(geom), ...
%     coder.typeof(0), []};
% dynamics_function = @FullStateDynamics;
% init_dynamics(dynamics_function, arg_types);
% 
% %% Setup reference trajectory
% t_knots = 0:15:100;
% testcase = Testcase(0, 1000, f_physics, t_knots, 0.01, 0.022);
% simulation_loop(testcase, dynamics_function, ...
%     'f_control', f_control, 'f_estimation', f_estimation);