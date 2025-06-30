clear;
clc;
addpath("natnetMatlab\");

load("inidata.mat");
deg_norm = 180/256;
sin_norm = 2/256;

%%
% client = tcpclient('169.254.170.51', 65432);
mytrack = natnet();
mytrack.connect();
pause(0.1);

position = zeros(num,3);
dists = zeros(num,2);
losecount = 200;

while true

    if losecount > 40
        try
            disp('waiting for connection');
            client = tcpclient('169.254.170.51', 65432);
            losecount = 0;
        catch
            continue;
        end
    end

    tic;
    frame = mytrack.getFrameLabeledMarker();
    if frame.MarkerCount ~= num
        % error('track lost');
        disp('track lost');
    else
        for i = 1:num
            position(i,:) = [frame.LabeledMarker(markerlist(i)).X frame.LabeledMarker(markerlist(i)).Y frame.LabeledMarker(markerlist(i)).Z];
        end

        v11 = position(1,:) - ini_orig_pos(1,:);
        v12 = position(2,:) - position(1,:);
        v21 = position(3,:) - ini_orig_pos(2,:);
        v22 = position(4,:) - position(3,:);
        deg1 = vecangle360(v11,v12,[0 0 1]);
        deg2 = vecangle360(v21,v22,[0 0 1]);
        % sin1 = sind(deg1);
        % sin2 = sind(deg2);

        dataout = 127 + [deg1 deg2]/deg_norm;
        % dataout = 127 + [sin1 sin2]/sin_norm;
    end
    disp(dataout);
    % t = toc;
    % disp(t);
    try
        write(client, dataout, "uint8");
        losecount = 0;
    catch
        disp(['loseconnect_' num2str(losecount)]);
        losecount = losecount + 1;
    end
    pause(0.01-toc);
end

function a = vecangle360(v1,v2,n)
x = cross(v1,v2);
c = sign(dot(x,n))*norm(x);
a = atan2d(c,dot(v1,v2));
end
