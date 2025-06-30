close all;

tau = squeeze(out_train.yout{1}.Values.totaltorque.Data);
states = squeeze(out_train.yout{3}.Values.Data);

dt = 0.1;
ttotal = 20;

% idx = find(states(6,:) ~= 0);
% tstart = idx(1) + 10;

tstart = 1991;

tend = tstart + ttotal/samplingrate;
SS = states(1:7, tstart:tend);
SS(4:5,:) = states(2:3, [tstart:tend] + dt/samplingrate);
tar = cell(3,1);
tar{1} = tau(tstart:tend);
tar{2} = SS(4,:)';
tar{3} = SS(5,:)';

train_output = cell(3,1);
train_target = cell(3,1);
Wout = cell(3,1);
Woutmat = [];

lambda = 2e0; % 1e-4 2e0

for i = 1:3
    if i == 1
        % X = SS(1:5,:); % noreservoir
        X = SS;
        Y = tar{i};

        Wout{i} = Y'*X'*inv(X*X'+lambda*eye(size(X,1))); 
        % Wout{i} = [Wout{i}(1:5) 0 0 zeros(1,4)]; % noreservoir
        Woutmat = [Woutmat; Wout{i}];
    else
        % X = SS([1:3 6:7],:); % noreservoir
        X = SS([1:3 6:end],:);
        Y = tar{i};

        Wout{i} = Y'*X'*inv(X*X'+lambda*eye(size(X,1))); 
        % Wout{i} = [Wout{i}(1:3) 0 0 Wout{i}(4:5) zeros(1,4)]; % noreservoir
        Wout{i} = [Wout{i}(1:3) 0 0 Wout{i}(4:end)];
        Woutmat = [Woutmat; Wout{i}];
    end

    train_output{i} = Wout{i}*SS;
    train_output{i} = train_output{i}';
    train_target{i} = tar{i};
end

figsize = [850 550 400 400;
    1300 550 400 400;
    850 50 400 400;
    1300 50 400 400];

figure('Color',[1 1 1]);
set(gcf,'position', figsize(1,:));
plot(SS');

figure('Color',[1 1 1]);
set(gcf,'position', figsize(3,:));
plot(train_target{1});
hold on;
plot(train_output{1});

figure('Color',[1 1 1]);
set(gcf,'position', figsize(4,:));
plot(train_target{2});
hold on;
plot(train_output{2});

% % figure('Color',[1 1 1]);
% % set(gcf,'position', figsize(4,:));
% % plot(train_target{3});
% % hold on;
% % plot(train_output{3});

Wout = Woutmat;
save('controlparam.mat', "Wout");