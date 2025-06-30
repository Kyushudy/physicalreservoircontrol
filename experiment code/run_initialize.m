clear;
clc;
addpath("natnetMatlab\");

mytrack = natnet();
mytrack.connect();
pause(0.1);

frame = mytrack.getFrameLabeledMarker();

%%
markerlist = [1 2 4 3];

num = length(markerlist);

frame = mytrack.getFrameLabeledMarker();

ini_pos = zeros(num,3);
for i = 1:num
    ini_pos(i,:) = [frame.LabeledMarker(markerlist(i)).X frame.LabeledMarker(markerlist(i)).Y frame.LabeledMarker(markerlist(i)).Z];
end

v1 = ini_pos(1,:) - ini_pos(2,:);
v2 = ini_pos(3,:) - ini_pos(4,:);

ini_orig_pos = [ini_pos(1,:) + v1*4/round(v1(2)*100);
    ini_pos(3,:) + v2*4/round(v2(2)*100)];


save("inidata.mat", "markerlist", "num", "ini_orig_pos", "ini_pos");
