clc; clear; close all;

%% Load cropped image
img = imread('1 km region.png');
img = im2double(img);

[rows, cols, ~] = size(img);
total_pixels = rows * cols;

%% Define reference colors and values

hex_colors = {
    '#f9fb0e'
    '#fad32a'
    '#f1b94a'
    '#b7bd64'
    '#7cbf7b'
    '#38b99e'
    '#07a9c2'
    '#0c93d2'
    '#0d75dc'
    '#2053d4'
    '#362b88'
};

values = [1 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1 0.0]';

num_ref = length(values);
ref_rgb = zeros(num_ref,3);

for k = 1:num_ref
    hex = hex_colors{k}(2:end);
    ref_rgb(k,1) = hex2dec(hex(1:2));
    ref_rgb(k,2) = hex2dec(hex(3:4));
    ref_rgb(k,3) = hex2dec(hex(5:6));
end

ref_rgb = ref_rgb / 255;

%% Flatten pixels
pixels = reshape(img,[],3);
num_pixels = size(pixels,1);

%% Compute distances (vectorized)

distances = zeros(num_pixels,num_ref);

for k = 1:num_ref
    diff_rgb = pixels - ref_rgb(k,:);
    distances(:,k) = sqrt(sum(diff_rgb.^2,2));
end

%% Inverse-distance weighting

% Prevent divide-by-zero
distances(distances < 1e-8) = 1e-8;

weights = 1 ./ distances;
weights = weights ./ sum(weights,2);

estimated_value = weights * values;

%% Histogram binning

bin_edges = linspace(0,1,11);  % 0,0.1,...,1

counts = histcounts(estimated_value, bin_edges);
percent = 100 * counts / total_pixels;

%% Display distribution (high to low)

fprintf('\nSun Visibility Distribution:\n\n');

for k = length(percent):-1:1
    fprintf('%.1f – %.1f  -->  %.2f %%\n', ...
        bin_edges(k), bin_edges(k+1), percent(k));
end

fprintf('\nCheck: Sum = %.2f %%\n', sum(percent));

%% Compute mean visibility directly (better than midpoint approximation)

v_mean = mean(estimated_value);

fprintf('\nMean Visibility = %.4f\n', v_mean);

%% Compute average solar flux

G_sc = 1361;   % W/m^2
G_avg = G_sc * v_mean;

fprintf('Average Solar Flux = %.2f W/m^2\n', G_avg);

