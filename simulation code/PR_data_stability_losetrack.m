clear;
clc;
close all;

addpath('./rc/');

load("success_l1_8_l2_10.mat");

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

%% control
iscontrol = true;
planactivate = true;
controlactivate = true;
robot_ini_theta_p = 0;
robot_ini_thetadot_p = 0;
begintime = 0;
control_ref = [pi; 0];
stoptime = 60; % 30
[SinusoidBus, busin] = create_bus_input(control_ref, dt);

%% simulation
start_list = 0.5:0.5:25;
period_list = 0.2:0.2:5;

% start_list = 8;
% period_list = 5;
list = [];

for res_losetrack_start = start_list
    for res_losetrack_period = period_list
        list = [list; res_losetrack_start, res_losetrack_period];
    end
end

data = cell(100, 2);

for i = 1:size(list,1) % size(list,1)
    disp(['running code ' num2str(i) '/' num2str(size(list,1))]);
    ep = ceil(i/100);
    idx = i - 100*(ep-1);

    ifreslosetrack = true;
    res_losetrack_start = list(i,1); % s
    res_losetrack_period = list(i,2); % s
    try
        controloutput_swingup = rc.robotcontrol(model_name);
    catch
        controloutput_swingup = [];
    end
    data{idx,1} = list(i,:);
    data{idx,2} = controloutput_swingup;
    if mod(idx,10) == 0
        save(['data\data_losetrack_ep' num2str(ep) '.mat'], "data");
        Simulink.sdi.clear;
    end
end

%% function
function [SinusoidBus, busin] = create_bus_input(input, dt)
    inputDim = 2;
    
    [~, numy] = size(input);

    time = 0:dt:(numy-1)*dt;

    clear elems;
    for j = 1:inputDim
        elems(j) = Simulink.BusElement;
        elems(j).Name = ['x' num2str(j)];
    end
    SinusoidBus = Simulink.Bus;
    SinusoidBus.Elements = elems;
    clear busin;
    busin.x1 = timeseries(input(1,:)',time');
    busin.x2 = timeseries(input(2,:)',time');

end