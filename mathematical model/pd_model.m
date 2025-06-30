clear;
clc;
close all;

simfile = "success_l1_8_l2_10.mat";
expfile = "vid_controlparam.mat";

wscale = -256/pi;
x = 0.04:0.02:0.14;
y = 0.001*[4.77 6.94 9.41 11.59 14.13 16.53];
p = polyfit(x, y, 1);

%%

% editable reservoir
l_list = [0.08; 0.10];
lt_list = [0.04; 0.04];
param_l = [lt_list, l_list]; % 10%~1000%
mp_list = polyval(p, l_list);
mp = [[0.0086; 0.0086], mp_list];
mb = 0.003;
mo = [0.0058, 0.0041;
    0.0058, 0.0041];
Ip = [0.00000337, mp(1,2)*param_l(1,2)*param_l(1,2)/12;
    0.00000337, mp(2,2)*param_l(2,2)*param_l(2,2)/12];
cp = [[0.009; 0.009], l_list/2];
param_c = (cp.*mp + 0.*mb + param_l.*mb + param_l.*mo)./(mp + mb + mb + mo); % 10%~1000%
param_I = Ip + mp.*(param_c - cp).*(param_c - cp) + mb.*param_c.*param_c + mb.*(param_l - param_c).*(param_l - param_c) + mo.*(param_l - param_c).*(param_l - param_c); % 10%~1000%
param_m = mp + 2*mb + mo; % 10%~1000%
param_mu = 4.4e-5*ones(2,2); % 10%~1000%

shift_mat = {[1 0.5; 1 0.5], [1 1.5; 1 0.5], [1 0.5; 1 1.5], [1 1.5; 1 1.5]};

param_l_list = {shift_mat{1}.*param_l, shift_mat{2}.*param_l, shift_mat{3}.*param_l, shift_mat{4}.*param_l, param_l};
param_c_list = {shift_mat{1}.*param_c, shift_mat{2}.*param_c, shift_mat{3}.*param_c, shift_mat{4}.*param_c, param_c};
param_I_list = {shift_mat{1}.*param_I, shift_mat{2}.*param_I, shift_mat{3}.*param_I, shift_mat{4}.*param_I, param_I};
param_m_list = {shift_mat{1}.*param_m, shift_mat{2}.*param_m, shift_mat{3}.*param_m, shift_mat{4}.*param_m, param_m};
param_mu_list = {shift_mat{1}.*param_mu, shift_mat{2}.*param_mu, shift_mat{3}.*param_mu, shift_mat{4}.*param_mu, param_mu};

% editable robot
robot_penrho = 1000; 
robot_penlength = 0.20; % 10%~1000%
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_damping_coeff = 1e-5; % 10%~1000%
robot_m = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth; % 10%~1000%

shift_list = [0.5:0.1:1.5 1];
robot_penlength_list = shift_list.*robot_penlength;
robot_damping_coeff_list = shift_list.*robot_damping_coeff;
robot_m_list = shift_list.*robot_m;

% linearize base
robot_base_state = [0; 0];
robot_target_state = [0; 0];
amp_factor = [0.15; 0.03]; 
res_base_state = [(robot_base_state.*amp_factor)';
    -(robot_base_state.*amp_factor)';
    0 0;
    0 0];
kp = 10;

load(simfile,"rc");
Wout = 0*rc.Woutmat(1,:);
pd_parameters;

sysrob = ss(A_rob, B_rob, C_rob, D_rob);
polesrob_0 = pole(sysrob);

systrain = ss(A_sys, B_sys, C_sys, D_sys);
polestrain_0 = pole(systrain);

load(simfile,"rc");
polesplan_sim_0 = [];
for i = 0:0.01:1 % 0.0016
    % Wout = rc.Woutmat(1,:); % balancing
    Wout = i*(rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
    pd_parameters;
    
    sys = ss(A_sys, B_sys, C_sys, D_sys);
    polesplan_sim_0 = [polesplan_sim_0 pole(sys)];
end
polesplan_sim_0 = sort(polesplan_sim_0, 'descend');

load(expfile);
Wout(:,6:7) = Wout(:,6:7).*wscale;
Woutmat = Wout;
polesplan_exp_0 = [];
for i = 0:0.01:1 % 0.011
    % Wout = Woutmat(1,:); % balancing
    Wout = i*(Woutmat(1,:) + Woutmat(1,4)*Woutmat(2,:) + Woutmat(1,5)*Woutmat(3,:)); % planning
    pd_parameters;
    
    sys = ss(A_sys, B_sys, C_sys, D_sys);
    polesplan_exp_0 = [polesplan_exp_0 pole(sys)];
end
polesplan_exp_0 = sort(polesplan_exp_0, 'descend');

load(simfile,"rc");
polesplan_sim_0_perturbres = [];
param_l = param_l_list{end};
param_c = param_c_list{end};
param_I = param_I_list{end};
param_m = param_m_list{end};
param_mu = param_mu_list{end};
num = length(param_l_list)-1;
for i1 = 1:num
    for i2 = 1:num
        for i3 = 1:num
            for i4 = 1:num
                for i5 = 1:num
                    param_l = param_l_list{i1};
                    param_c = param_c_list{i2};
                    param_I = param_I_list{i3};
                    param_m = param_m_list{i4};
                    param_mu = param_mu_list{i5};
                
                    % Wout = rc.Woutmat(1,:); % balancing
                    Wout = (rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
                    pd_parameters;
                    
                    sys = ss(A_sys, B_sys, C_sys, D_sys);
                    polesplan_sim_0_perturbres = [polesplan_sim_0_perturbres pole(sys)];
                end
            end
        end
    end
end

figure('Color',[1 1 1]);
plot(real(polesplan_sim_0_perturbres),imag(polesplan_sim_0_perturbres),".k");
hold on;
plot(real(polesplan_sim_0(:,end)),imag(polesplan_sim_0(:,end)),".b");
hold on;
plot(real(polesplan_exp_0),imag(polesplan_exp_0),".r");

load(simfile,"rc");
polesplan_sim_0_perturbrob = [];
robot_penlength = robot_penlength_list(end);
robot_damping_coeff = robot_damping_coeff_list(end);
robot_m = robot_m_list(end);
num = length(robot_penlength_list)-1;
for i1 = 1:num
    for i2 = 1:num
        for i3 = 1:num
            robot_penlength = robot_penlength_list(i1);
            robot_damping_coeff = robot_damping_coeff_list(i2);
            robot_m = robot_m_list(i3);
        
            % Wout = rc.Woutmat(1,:); % balancing
            Wout = (rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
            pd_parameters;
            
            sys = ss(A_sys, B_sys, C_sys, D_sys);
            polesplan_sim_0_perturbrob = [polesplan_sim_0_perturbrob pole(sys)];

        end
    end
end

figure('Color',[1 1 1]);
plot(real(polesplan_sim_0_perturbrob),imag(polesplan_sim_0_perturbrob),".k");
hold on;
plot(real(polesplan_sim_0(:,end)),imag(polesplan_sim_0(:,end)),".b");
hold on;
plot(real(polesplan_exp_0),imag(polesplan_exp_0),".r");

%%

% editable reservoir
l_list = [0.08; 0.10];
lt_list = [0.04; 0.04];
param_l = [lt_list, l_list]; % 10%~1000%
mp_list = polyval(p, l_list);
mp = [[0.0086; 0.0086], mp_list];
mb = 0.003;
mo = [0.0058, 0.0041;
    0.0058, 0.0041];
Ip = [0.00000337, mp(1,2)*param_l(1,2)*param_l(1,2)/12;
    0.00000337, mp(2,2)*param_l(2,2)*param_l(2,2)/12];
cp = [[0.009; 0.009], l_list/2];
param_c = (cp.*mp + 0.*mb + param_l.*mb + param_l.*mo)./(mp + mb + mb + mo); % 10%~1000%
param_I = Ip + mp.*(param_c - cp).*(param_c - cp) + mb.*param_c.*param_c + mb.*(param_l - param_c).*(param_l - param_c) + mo.*(param_l - param_c).*(param_l - param_c); % 10%~1000%
param_m = mp + 2*mb + mo; % 10%~1000%
param_mu = 4.4e-5*ones(2,2); % 10%~1000%

shift_mat = {[1 0.5; 1 0.5], [1 1.5; 1 0.5], [1 0.5; 1 1.5], [1 1.5; 1 1.5]};

param_l_list = {shift_mat{1}.*param_l, shift_mat{2}.*param_l, shift_mat{3}.*param_l, shift_mat{4}.*param_l, param_l};
param_c_list = {shift_mat{1}.*param_c, shift_mat{2}.*param_c, shift_mat{3}.*param_c, shift_mat{4}.*param_c, param_c};
param_I_list = {shift_mat{1}.*param_I, shift_mat{2}.*param_I, shift_mat{3}.*param_I, shift_mat{4}.*param_I, param_I};
param_m_list = {shift_mat{1}.*param_m, shift_mat{2}.*param_m, shift_mat{3}.*param_m, shift_mat{4}.*param_m, param_m};
param_mu_list = {shift_mat{1}.*param_mu, shift_mat{2}.*param_mu, shift_mat{3}.*param_mu, shift_mat{4}.*param_mu, param_mu};

% editable robot
robot_penrho = 1000; 
robot_penlength = 0.20; % 10%~1000%
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_damping_coeff = 1e-5; % 10%~1000%
robot_m = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth; % 10%~1000%

shift_list = [0.5:0.1:1.5 1];
robot_penlength_list = shift_list.*robot_penlength;
robot_damping_coeff_list = shift_list.*robot_damping_coeff;
robot_m_list = shift_list.*robot_m;

% linearize base
robot_base_state = [pi; 0];
robot_target_state = [pi; 0];
amp_factor = [0.15; 0.03]; 
res_base_state = [(robot_base_state.*amp_factor)';
    -(robot_base_state.*amp_factor)';
    0 0;
    0 0];
kp = 10;

load(simfile,"rc");
Wout = 0*rc.Woutmat(1,:);
pd_parameters;

sysrob = ss(A_rob, B_rob, C_rob, D_rob);
polesrob_pi = pole(sysrob);

systrain = ss(A_sys, B_sys, C_sys, D_sys);
polestrain_pi = pole(systrain);

load(simfile,"rc");
polesstab_sim_pi = [];
for i = 0:0.01:1 % 0.1432
    Wout = i*rc.Woutmat(1,:); % balancing
    % Wout = i*(rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
    pd_parameters;
    
    sys = ss(A_sys, B_sys, C_sys, D_sys);
    polesstab_sim_pi = [polesstab_sim_pi pole(sys)];
end
polesstab_sim_pi = sort(polesstab_sim_pi, 'descend');

load(expfile);
Wout(:,6:7) = Wout(:,6:7).*wscale;
Woutmat = Wout;
polesstab_exp_pi = [];
for i = 0:0.01:1 % 0.1157
    Wout = i*Woutmat(1,:); % balancing
    % Wout = i*(Woutmat(1,:) + Woutmat(1,4)*Woutmat(2,:) + Woutmat(1,5)*Woutmat(3,:)); % planning
    pd_parameters;
    
    sys = ss(A_sys, B_sys, C_sys, D_sys);
    polesstab_exp_pi = [polesstab_exp_pi pole(sys)];
end
polesstab_exp_pi = sort(polesstab_exp_pi, 'descend');

load(simfile,"rc");
polesstab_sim_pi_perturbres = [];
param_l = param_l_list{end};
param_c = param_c_list{end};
param_I = param_I_list{end};
param_m = param_m_list{end};
param_mu = param_mu_list{end};
num = length(param_l_list)-1;
for i1 = 1:num
    for i2 = 1:num
        for i3 = 1:num
            for i4 = 1:num
                for i5 = 1:num
                    param_l = param_l_list{i1};
                    param_c = param_c_list{i2};
                    param_I = param_I_list{i3};
                    param_m = param_m_list{i4};
                    param_mu = param_mu_list{i5};
                
                    Wout = rc.Woutmat(1,:); % balancing
                    % Wout = (rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
                    pd_parameters;
                    
                    sys = ss(A_sys, B_sys, C_sys, D_sys);
                    polesstab_sim_pi_perturbres = [polesstab_sim_pi_perturbres pole(sys)];
                end
            end
        end
    end
end

figure('Color',[1 1 1]);
plot(real(polesstab_sim_pi_perturbres),imag(polesstab_sim_pi_perturbres),".k");
hold on;
plot(real(polesstab_sim_pi(:,end)),imag(polesstab_sim_pi(:,end)),".b");
hold on;
plot(real(polesstab_exp_pi),imag(polesstab_exp_pi),".r");

load(simfile,"rc");
polesstab_sim_pi_perturbrob = [];
robot_penlength = robot_penlength_list(end);
robot_damping_coeff = robot_damping_coeff_list(end);
robot_m = robot_m_list(end);
num = length(robot_penlength_list)-1;
for i1 = 1:num
    for i2 = 1:num
        for i3 = 1:num
            robot_penlength = robot_penlength_list(i1);
            robot_damping_coeff = robot_damping_coeff_list(i2);
            robot_m = robot_m_list(i3);
        
            Wout = rc.Woutmat(1,:); % balancing
            % Wout = (rc.Woutmat(1,:) + rc.Woutmat(1,4)*rc.Woutmat(2,:) + rc.Woutmat(1,5)*rc.Woutmat(3,:)); % planning
            pd_parameters;
            
            sys = ss(A_sys, B_sys, C_sys, D_sys);
            polesstab_sim_pi_perturbrob = [polesstab_sim_pi_perturbrob pole(sys)];

        end
    end
end

figure('Color',[1 1 1]);
plot(real(polesstab_sim_pi_perturbrob),imag(polesstab_sim_pi_perturbrob),".k");
hold on;
plot(real(polesstab_sim_pi(:,end)),imag(polesstab_sim_pi(:,end)),".b");
hold on;
plot(real(polesstab_exp_pi),imag(polesstab_exp_pi),".r");