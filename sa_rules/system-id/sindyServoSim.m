% File            : sindyServoSim.m
% Description     : This file is responsible for Creating a model using SINDy
%                 : Based off of the examples in MATAVEID
% Date            : 10.02.2025
% Last Modified   : 10.02.2025

clc; clear; close all;

% Load CSV data
file = fullfile('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/servo_output.csv');
X = csvread(file);
file = fullfile('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/sim_state.csv');
Y = csvread(file);
file = fullfile('/mnt/oldhd/Data/PGFUZZ-data/mav_tlogs/servo_experiments/local_position.csv');
Z = csvread(file);

inputs = X(:,3:6);
outputs = [Y(:,1:3),zeros(length(Y),1)];
time = X(:,1);
sampleTime = 1/4000; % 4000Hz (From the scheduler loop)
% TODO: Do filtering of outputs
% noiseThreshold = 0.1;
% outputs_filt = mi.filtfilt(outputs', time', noiseThreshold)';

% Sindy - Sparce identification Dynamics
degree = 2;
lambda = 0.05;
[output_val] = mi.sindy(inputs,outputs,degree,lambda,sampleTime);
%
[x_up, t] = mc.nlsim(fx_up, u_up, x0, stepTime, 'ode15s'); 
% % Compare
figure
plot([x_up x_down])
hold on
plot(y(1:100:end));
legend('Simulation', 'Measurement')
ylabel('Rotation')
xlabel('Time')
grid on

