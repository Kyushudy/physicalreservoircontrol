clear;
clc;
close all;

datacell = [];

for ep = 1:6
    load(['data\data_noise_ep' num2str(ep) '.mat']);
    datacell = [datacell; data];
end

for i = 1:size(datacell,1)
    simout = datacell{i,2};
    states = squeeze(simout.yout{3}.Values.Data);
    havedata = sum(states(6,:) ~= 0);
    datacell{i,3} = havedata;
end