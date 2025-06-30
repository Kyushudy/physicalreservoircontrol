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
robot_var_list = [0 logspace(-6,-2,20)];
res_var_list = [0 logspace(-5,-1,20)];
list = [];

for robot_noise_variance = robot_var_list
    for res_noise_variance = res_var_list
        for i = 1:10
            robot_noise_seed = round(10000*rand());
            res_noise_seed = round(10000*[rand() rand()]);
            list = [list; robot_noise_variance, robot_noise_seed, res_noise_variance, res_noise_seed];
        end
    end
end

data = cell(100, 2);
% load('data\data_noise_ep25.mat');

for i = 1:size(list,1) % size(list,1)
    disp(['running code ' num2str(i) '/' num2str(size(list,1))]);
    ep = ceil(i/100);
    idx = i - 100*(ep-1);

    ifrobotnoise = true;
    robot_noise_variance = list(i,1); % N 0.01~0.00001 logspace(-5,-2,20);
    robot_noise_seed = list(i,2);

    ifresnoise = true;
    res_noise_variance = [list(i,3) list(i,3)]; % rad 0.1~0.0001 logspace(-4,-1,20);
    res_noise_seed = [list(i,4) list(i,5)];

    try
        controloutput_swingup = rc.robotcontrol(model_name);
    catch
        controloutput_swingup = [];
    end
    data{idx,1} = list(i,:);
    data{idx,2} = controloutput_swingup;
    if mod(idx,10) == 0
        save(['data\data_noise_ep' num2str(ep) '.mat'], "data");
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