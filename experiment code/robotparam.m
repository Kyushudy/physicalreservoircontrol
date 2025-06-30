clear;
clc;

stoptime = 120;

g = 9.80665;

robot_penrho = 1000;
robot_penlength = 0.20;
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_penposz = 0.3;
robot_sphere_mass = 0.1;
robot_sphere_radius = 0.015;
robot_damping_coeff = 1e-5; % 1e-5 or 0
torquelim = 0.25*robot_penrho*robot_penlength*robot_penwidth*robot_pendepth*robot_penlength/2*g; % 1 for fully acutated <0.08 for underactuated 0.02
% torquelim = 1;
I_est = 1e-3; % 1e-2
switch_threshold = pi/4; % pi/4

robot_ini_theta_p = pi-0.1;
robot_ini_thetadot_p = 0;

Wout = 0*ones(3,1+4+2);

ref_theta = 0;
ref_thetadot = 0;

samplingrate = 0.01;

amp_factor = [0.15; 0.03]/pi*180; 

motor1mid = 0.445*180;
motor2mid = 0.45*180;

%% stability
ifrobotkicking = false;
robot_kick_start = 10; % s
robot_kick_period = 0.1; % s
robot_kick_height = 0; % N

robot_pulse_start = 60; % s
robot_pulse_period = 0.1*100; % s/%
robot_pulse_freq = 0.05; % Hz
robot_pulse_hight = 0.5; % N

ifrobotnoise = false;
robot_noise_variance = 0.0001; % N 0.01~0.00001 logspace(-5,-2,20);
robot_noise_seed = round(10000*rand());

ifresnoise = false;
res_noise_variance = 0.01*[1 1]; % rad 0.1~0.0001 logspace(-4,-1,20);
res_noise_seed = round(10000*[rand() rand()]);

ifreslosetrack = false;
res_losetrack_start = 9;
res_losetrack_period = 5;