% Wheel Torque Calculations
SF = 1.2
width = 0.07; % m
diameter = 0.2; % m
drawbarForce = 4.2 % N, Pull from terramechanics trade study
roverMass = 16 % kg
g = 1.62 % m/s^2
wheelCount = 4
v = 0.01 % m/s
aMax = drawbarForce/roverMass
mu = 0.6 % Taken from trade study, worse case scenario assumed.
wheelLoad = roverMass*g/wheelCount
MTT = mu*(diameter/2)*wheelLoad*SF %N/m