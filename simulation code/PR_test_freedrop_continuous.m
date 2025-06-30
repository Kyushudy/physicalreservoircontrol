clear;
clc;
close all;

addpath('./rc/');

penrho = 1174*ones(2,2);
penlength = [0.04 0.08;
    0.04 0.10];
% penlength = [0.04 0.10 0.04;
%     0.04 0.06 0.12];
penwidth = 0.02*ones(2,2);
pendepth = 0.005*ones(2,2);
penposx = [0.25 0;
    0.35 0];
penposz = [0.15 0;
    0.15 0];
sphere_mass = [(0.003+0.0058)*ones(2,2) (0.003+0.0041)*ones(2,1)];
sphere_radius = 0.015*ones(2,3);
damping_coeff = 1e-4*ones(2,2); % 4.4e-5*ones(2,2)
ini_theta_p = 0*ones(2,2);
ini_thetadot_p = 0*ones(2,2);
amp_factor = [0.15; 0.03]; 
Win = eye(2);

g = 9.80665;  

robot_penrho = 1000;
robot_penlength = 0.20;
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_penposz = 0; % 0.3
robot_sphere_mass = 0.1;
robot_sphere_radius = 0.015;
robot_damping_coeff = 1e-5; % 1e-5 or 0
torquelim = 0.25*robot_penrho*robot_penlength*robot_penwidth*robot_pendepth*robot_penlength/2*g; % 1 for fully acutated <0.08 for underactuated 0.02
% torquelim = 0.09;
I_est = 1e-3; % 1e-3, real 0.00106933
switch_threshold = pi/4; % pi/4

%% stability
ifrobotkicking = false;
robot_kick_start = 50; % s
robot_kick_period = 0.1; % s
robot_kick_height = 1; % N*m

robot_pulse_start = 30; % s
robot_pulse_period = 0.1*100; % s/%
robot_pulse_freq = 0.01; % Hz
robot_pulse_hight = 0.08; % N*m 0.022 0.05 1

ifrobotnoise = false;
robot_noise_variance = 0.0001; % N 0.01~0.00001 logspace(-5,-2,20);
robot_noise_seed = round(10000*rand());

ifresnoise = false;
res_noise_variance = 0.01*[1 1]; % rad 0.1~0.0001 logspace(-4,-1,20);
res_noise_seed = round(10000*[rand() rand()]);

ifreslosetrack = false;
res_losetrack_start = 9;
res_losetrack_period = 5;

ifreskicking = false;
res_kick_start = [20 30; 20 30]; % s
res_kick_period = 0.1*ones(2,2); % s
res_kick_height = [0 0; 0.032 0]; % N 0.005 0.015 0.021365; 0.005 0.015 0.032

%%

model_name = "PR_planningsim_2joint.slx";
dt = 0.1;
stoptime = 20;
samplingrate = 0.01;
washout = 0.005*stoptime/samplingrate;

%% loop begin
for dtheta = 0.1 % 0.01:0.01:pi/2
    % try
        rc = RC_trainbysim('sizeinput', 4, 'regularization', 2e-0, 'delaylen', dt, ...
            'sizereadout', 1+4+2, 'sizeoutput', 3, 'timestep', samplingrate, 'ifxnorm', true);
        rc.clearrecord();

        iscontrol = false;
        planactivate = true;
        controlactivate = false;
        robot_ini_theta_p = pi-0.1; % pi-0.1
        robot_ini_thetadot_p = 0;
        ini_theta_p = [0 0; 0 0];
        begintime = 0;
        control_ref = [1; 0];
        stoptime = 20; % 20
        [SinusoidBus, busin] = create_bus_input(control_ref, dt);
        
        %%
        [traindata] = rc.traindatacollect(model_name, washout);
        %%
        
        [train_output, train_target] = rc.train('planning', true);
        
        %% swingup+balancing
        % try
            ifrobotkicking = false;
            ifrobotnoise = false;
            ifresnoise = false;
            ifreslosetrack = false;
            ifreskicking = false;
            iscontrol = true;
            planactivate = true;
            controlactivate = true;
            robot_ini_theta_p = 0;
            robot_ini_thetadot_p = 0;
            begintime = 0;
            control_ref = [pi; 0];
            stoptime = 20; % 60
            [SinusoidBus, busin] = create_bus_input(control_ref, dt);

            controloutput_swingup = rc.robotcontrol(model_name);
        % catch
        %     controloutput_swingup = [];
        % end

    %     save(['data\PR_freedrop_dtheta' num2str(round(dtheta*100)) '.mat'], "rc", "controloutput_balancing", "controloutput_swingup");
    % catch
    % end
    % Simulink.sdi.clear;
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