clear;
clc;
close all;

%% run control

robotparam();
stoptime = 80;

robot_ini_theta_p = 0;
robot_ini_thetadot_p = 0;

ref_theta = pi;
ref_thetadot = 0;

load("records\vid_controlparam.mat");

%% stability
ifrobotkicking = false;
robot_kick_start = 10; % s
robot_kick_period = 0.1; % s
robot_kick_height = 0; % N

robot_pulse_start = 10; % s
robot_pulse_period = 0.1*100; % s/%
robot_pulse_freq = 0.5; % Hz
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

%% simulation
start_list = 20 + (1:1:25);
period_list = 0.5:0.5:5;

% start_list = 45;
% period_list = 5;
list = [];

for res_losetrack_start = start_list
    for res_losetrack_period = period_list
        list = [list; res_losetrack_start, res_losetrack_period];
    end
end

data = cell(100, 3);

for i = 1:size(list,1) % size(list,1)
    disp(['running code ' num2str(i) '/' num2str(size(list,1))]);
    ep = ceil(i/100);
    idx = i - 100*(ep-1);

    ifreslosetrack = true;
    res_losetrack_start = list(i,1); % s
    res_losetrack_period = list(i,2); % s

    out_control = sim("PR_data.slx");
    data{idx,1} = list(i,:);
    data{idx,2} = out_control;
    states = squeeze(out_control.yout{3}.Values.Data);
    havedata = sum(states(6,:) ~= 0);
    data{idx,3} = havedata;
    if mod(idx,10) == 0 || i == size(list,1)
        save(['data\data_losetrack_ep' num2str(ep) '.mat'], "data");
        Simulink.sdi.clear;
    end
    pause(2);
end