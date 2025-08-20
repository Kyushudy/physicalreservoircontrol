%%
robot_base_state = [0; 0];
robot_target_state = [0; 0];
amp_factor = [0.15; 0.03]; 
res_base_state = [(robot_base_state.*amp_factor)';
    -(robot_base_state.*amp_factor)';
    0 0;
    0 0];
Wout = zeros(1,7);
kp = 10;

%%
wscale = 256/pi;
x = 0.04:0.02:0.14;
y = 0.001*[4.77 6.94 9.41 11.59 14.13 16.53];
p = polyfit(x, y, 1);
l_list = [0.08; 0.10];
lt_list = [0.04; 0.04];
param_l = [lt_list, l_list];
mp_list = polyval(p, l_list);
mp = [[0.0086; 0.0086], mp_list];
mb = 0.003;
mo = [0.0058, 0.0041;
    0.0058, 0.0041];
Ip = [0.00000337, mp(1,2)*param_l(1,2)*param_l(1,2)/12;
    0.00000337, mp(2,2)*param_l(2,2)*param_l(2,2)/12];
cp = [[0.009; 0.009], l_list/2];
param_c = (cp.*mp + 0.*mb + param_l.*mb + param_l.*mo)./(mp + mb + mb + mo);
param_I = Ip + mp.*(param_c - cp).*(param_c - cp) + mb.*param_c.*param_c + mb.*(param_l - param_c).*(param_l - param_c) + mo.*(param_l - param_c).*(param_l - param_c);
param_m = mp + 2*mb + mo;
param_mu = 4.4e-5*ones(2,2);

g = 9.80665;  

for i = 1:2
    mid_M11 = param_m(i,1)*param_c(i,1)^2 + param_I(i,1) + param_m(i,2)*(param_l(i,1)^2+param_c(i,2)^2+2*param_l(i,1)*param_c(i,2)*cos(res_base_state(2,i))) + param_I(i,2);
    mid_M12 = param_m(i,2)*param_l(i,1)*param_c(i,2)*cos(res_base_state(2,i)) + param_m(i,2)*param_c(i,2)^2 + param_I(i,2);
    mid_M21 = mid_M12;
    mid_M22 = param_m(i,2)*param_c(i,2)^2 + param_I(i,2);
    mid_h = param_m(i,2)*param_l(i,1)*param_c(i,2)*sin(res_base_state(2,i));
    mid_G11 = - g*param_m(i,1)*param_c(i,1)*cos(res_base_state(1,i)) - g*param_m(i,2)*param_l(i,1)*cos(res_base_state(1,i)) - g*param_m(i,2)*param_c(i,2)*cos(res_base_state(1,i)+res_base_state(2,i));
    mid_G12 = - g*param_m(i,2)*param_c(i,2)*cos(res_base_state(1,i)+res_base_state(2,i));
    mid_G21 = mid_G12;
    mid_G22 = mid_G12;
    mid_D1 = - g*param_m(i,1)*param_c(i,1)*sin(res_base_state(1,i)) - g*param_m(i,2)*param_l(i,1)*sin(res_base_state(1,i)) - g*param_m(i,2)*param_c(i,2)*sin(res_base_state(1,i)+res_base_state(2,i));
    mid_D2 = - g*param_m(i,2)*param_c(i,2)*sin(res_base_state(1,i)+res_base_state(2,i));
    mid_mu1 = param_mu(i,1);
    mid_mu2 = param_mu(i,2);

    PmatL{i} = [1 0 0 0;
        0 1 0 0;
        0 0 mid_M11 mid_M12;
        0 0 mid_M21 mid_M22];
    PmatR{i} = [0 0 1 0;
        0 0 0 1;
        mid_G11-kp mid_G12 -mid_mu1 0;
        mid_G21 mid_G22 0 -mid_mu2];
    PmatD{i} = [0;
        0;
        mid_D1;
        mid_D2];

    A_P{i} = PmatL{i}\PmatR{i};
    In_P{i} = PmatL{i}\PmatD{i};
end

B_P{1} = PmatL{1}\[0 0;
    0 0;
    kp*amp_factor(1) 0;
    0 0];

B_P{2} = PmatL{2}\[0 0;
    0 0;
    0 kp*amp_factor(2);
    0 0];

A_res = [A_P{1} zeros(4,4);
    zeros(4,4) A_P{2}];
B_res = [B_P{1};
    B_P{2}];
C_res = [0 Wout(1,6) 0 0 0 Wout(1,7) 0 0];
D_res = [Wout(1,2) Wout(1,3)];
Bias_res = Wout(1,1) + Wout(1,2:3)*robot_base_state + Wout(1,4:5)*robot_target_state;

robot_penrho = 1000;
robot_penlength = 0.20;
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_damping_coeff = 1e-5; % 1e-5 or 0
robot_m = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth;

RmatL = [1 0;
    0 1/3*robot_m*robot_penlength^2];
RmatR = [0 1;
    -robot_m*g*robot_penlength/2*cos(robot_base_state(1)) -robot_damping_coeff];
RmatD = [0;
    -robot_m*g*robot_penlength/2*sin(robot_base_state(1))];

A_rob = RmatL\RmatR;
B_rob = RmatL\[0; 1];
C_rob = [1 0;
    0 1];
D_rob = 0;

A_sys = [A_rob + B_rob*D_res*C_rob B_rob*C_res;
    B_res*C_rob A_res];
B_sys = [B_rob*Bias_res; zeros(8,1)];

C_sys = eye(10);
D_sys = 0;


%The stability of 3-DOF triple-rigid-body pendulum system near resonances
