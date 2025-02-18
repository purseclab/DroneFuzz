clear; clc; close all;

% Load data from files
% servo_data      = readmatrix('Trial/servo_output.csv');
% sim_state       = readmatrix('Trial/sim_state.csv');
% local_position  = readmatrix('Trial/local_position.csv');
servo_data      = readmatrix('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/servo_output.csv');
sim_state       = readmatrix('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/sim_state.csv');
local_position  = readmatrix('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/local_position.csv');

% Inputs: rotor speeds (Omega1..Omega4)
Y_inputs = servo_data(:, 3:6);

% Outputs: 9 signals (phi,theta,psi, p,q,r, x,y,z)
% According to your code, sim_state(:, [1:3, 4:6]) => [phi,theta,psi, p,q,r]
% and local_position(:, 5:7) => [x, y, z]
X_output = [sim_state(:,1:3), sim_state(:,4:6), local_position(:,5:7)];

% Sampling time
Ts = mean(diff(servo_data(:,1)));
data = iddata(X_output, Y_inputs, Ts);

% "Order = [ny, nu, nx]" => [9 outputs, 4 inputs, 12 states]
Order = [9, 4, 12];

% Initial parameter guess [m; Ix; Iy; Iz; l; b; d]
Parameters = {1.0, 0.01, 0.01, 0.02, 0.1, 1e-5, 1e-6};

% Now call idnlgrey with 5 positional arguments, THEN name-value pairs:
Model = idnlgrey('quadcopterEOM', Order, Parameters);

optnl = nlgreyestOptions;
optnl.Display = 'on';

% Estimate model parameters using data
Model_Est = nlgreyest(data, Model, optnl);

% Present the results
present(Model_Est);
