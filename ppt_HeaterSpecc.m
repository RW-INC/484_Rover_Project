clear; clc; close all;
sigma = 5.670374419e-8;
T_batt = -10 + 273.15;   % Kelvin, selected a value above minimum operating floor
T_env  = 30;            % Kelvin
Housing_dimensions = [0.08, 0.035, 0.05]; % Meters, Tweak based on size of Lion we need (18000mah for complete mission, smaller if using solar recharging)
A = 2*(Housing_dimensions(1)*Housing_dimensions(2)+Housing_dimensions(2)*Housing_dimensions(3))+(Housing_dimensions(1)*Housing_dimensions(3))                % Equation depends on battery mounting so adjust accordingly, currently assuming 1 face not radiating
hours_in_shadow = 72;    % Hours in shade according to MPA, assuming 3 days as a bad case scenario
time_seconds = hours_in_shadow * 3600;
materials = { ... %Testing a few common material emmisivities
    'Polished Aluminum', 0.05; ...
    'Anodized Aluminum', 0.77; ...
    'Stainless Steel', 0.60; ...
    'Black Paint (High-Emissivity)', 0.90; ...
    'MLI Blanket (Effective)', 0.03};
num_materials = size(materials,1);
heater_power = zeros(num_materials,1);
for i = 1:num_materials
    emissivity = materials{i,2};
    heater_power(i) = emissivity * sigma * A * (T_batt^4 - T_env^4);
end
material_names = materials(:,1);
figure;
bar(heater_power);
xticks(1:num_materials);
xticklabels(material_names);
xtickangle(45);
ylabel('Required Heater Power (W)');
title("Heater Power Required in South Pole Crater (" + T_env + " K) over " + hours_in_shadow + " hours of shade");
grid on;
fprintf('\nHeater Power Results:\n\n');
for i = 1:num_materials
    fprintf('%s: %.2f W\n', material_names{i}, heater_power(i));
end