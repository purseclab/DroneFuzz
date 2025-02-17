clear; clc; close all;

% Load data from files
servo_data      = readmatrix('Trial/servo_output.csv');
sim_state       = readmatrix('Trial/sim_state.csv');
local_position  = readmatrix('Trial/local_position.csv');

% Extract inputs and states
Y_inputs        = servo_data(:, 3:6);                      % Rotor speeds [Omega1, Omega2, Omega3, Omega4]
X_output        = [sim_state(:, [1:3, 4:6]), local_position(:, 5:7)];
% X_states        = [X_states,zeros(length(X_states),3)];

% Define sampling time
Ts              = mean(diff(servo_data(:, 1)));
data            = iddata(X_output, Y_inputs, Ts);

% Outputs | Inputs | States 
Order           = [9, 4, 12];

InitialStates = zeros(12, 1);
% Initial parameter guess [m; Ix; Iy; Iz; l; b; d]
Parameters      = {1.0; 0.01; 0.01; 0.02; 0.1; 1e-5; 1e-6};

% Create nonlinear grey-box model object
Model           = idnlgrey('quadcopterEOM', Order, Parameters, InitialStates);
% Estimate model parameters using data (optional)
Model_Est          = nlgreyest(data, Model);
