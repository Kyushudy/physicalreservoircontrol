clear;
clc;
addpath("natnetMatlab\");

[a, s1, s2] = initialize();
s = {s1, s2};
pin = {5, 6};
%%
motor1mid = 0.445*180;
motor2mid = 0.45*180;

writePosition(s1, motor1mid - 30, pin{1});
writePosition(s2, motor2mid - 30, pin{2});

pause(2);

writePosition(s1, motor1mid, pin{1});
writePosition(s2, motor2mid, pin{2});

%%
% client = tcpclient('169.254.170.51', 65431);

% tStart = tic;
% time = zeros(1000,1);
% j = 1;

losecount = 200;

while true

    if losecount > 40
        writePosition(s1, motor1mid, pin{1});
        writePosition(s2, motor2mid, pin{2});
        try
            disp('waiting for connection');
            client = tcpclient('169.254.170.51', 65431);
            losecount = 0;
        catch
            continue;
        end
    end

    if client.NumBytesAvailable > 0
        % time(j) = toc(tStart);
        % j = j + 1;
        datain = read(client, 2, "double");
        flush(client);
        disp(datain);
        for i = 1:2
            datain(i) = max(0, min(150, datain(i)));
            writePosition(s{i}, datain(i), pin{i});
        end
        losecount = 0;
    else
        disp(['loseconnect_' num2str(losecount)]);
        losecount = losecount + 1;
    end
end


function [a, s1, s2] = initialize()

a = arduino('COM4', 'UNO');
s1 = servo(a, 'D5');
s2 = servo(a, 'D6');
disp('arduino setup succeed!')

end