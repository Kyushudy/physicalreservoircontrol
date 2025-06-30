clear;
clc;
close all;

%% run training simulation

robotparam();
stoptime = 40;

out_train = sim("PR_data.slx");
save("outtrain.mat", "out_train");

%% train reservoir

robotparam();
% load("records\vid_outtrain.mat");
load("outtrain.mat", "out_train");
trainres();

pause(2);

%% run control

robotparam();
stoptime = 1000;

robot_ini_theta_p = 0;
robot_ini_thetadot_p = 0;

ref_theta = pi;
ref_thetadot = 0;

ifrobotkicking = true;
robot_kick_start = 1000+20; % s 50+20 100+20
robot_kick_period = 0.1; % s
robot_kick_height = 1; % N (1)

robot_pulse_start = 1000+20; % s 30+20 100+20
robot_pulse_period = 0.1*100; % s/%
robot_pulse_freq = 0.01; % Hz
robot_pulse_hight = 0.08; % N (0.08)

% load("records\vid_controlparam.mat");
% load("records\disturb_controlparam.mat");
load("controlparam.mat");

out_control = sim("PR_data.slx");
save("outcontrol.mat", "out_control");
load("outtrain.mat", "out_train");