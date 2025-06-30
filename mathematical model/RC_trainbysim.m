classdef RC_trainbysim < handle
% Echo State Network    
    properties
        lambda
        Wout
        Woutmat
        train_internalState
        train_reservoirReadout
        train_reservoirReadoutWashed
        train_reservoirTarget
        train_reservoirTargetWashed
        predict_internalState
        predict_reservoirReadout
        sizeinput
        sizereadout
        sizeoutput
        delaylen
        timestep
        ifxnorm
        xnorm
    end

    methods
        function rc = RC_trainbysim(varargin)
            
            rc.lambda = 1;
            rc.sizereadout = 1;
            rc.sizeoutput = 1;
            rc.sizeinput = 1;
            rc.train_reservoirReadoutWashed = [];
            
            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    case 'regularization', rc.lambda = varargin{i+1};
                    case 'sizeinput', rc.sizeinput = varargin{i+1};
                    case 'sizereadout', rc.sizereadout = varargin{i+1};
                    case 'sizeoutput', rc.sizeoutput = varargin{i+1};
                    case 'delaylen', rc.delaylen = varargin{i+1};
                    case 'timestep', rc.timestep = varargin{i+1};
                    case 'ifxnorm', rc.ifxnorm = varargin{i+1};
                    otherwise, error('the option does not exist');
                end
            end

            rc.xnorm = ones(rc.sizeinput,1);

            rc.train_reservoirTarget = cell(rc.sizeoutput,1);
            rc.train_reservoirTargetWashed = cell(rc.sizeoutput,1);
            for i = 1:rc.sizeoutput
                rc.train_reservoirTargetWashed{i} = [];
            end
            rc.Wout = cell(rc.sizeoutput,1);
            rc.Woutmat = zeros(rc.sizeoutput,rc.sizereadout);
        end

        function [] = clearrecord(rc, varargin)
            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    case 'regularization', rc.lambda = varargin{i+1};
                    case 'sizeinput', rc.sizeinput = varargin{i+1};
                    case 'sizereadout', rc.sizereadout = varargin{i+1};
                    case 'sizeoutput', rc.sizeoutput = varargin{i+1};
                    case 'delaylen', rc.delaylen = varargin{i+1};
                    case 'timestep', rc.timestep = varargin{i+1};
                    case 'ifxnorm', rc.ifxnorm = varargin{i+1};
                    otherwise, error('the option does not exist');
                end
            end

            rc.Wout = cell(rc.sizeoutput,1);
            rc.Woutmat = zeros(rc.sizeoutput,rc.sizereadout);

            rc.train_internalState = [];
            rc.train_reservoirReadout = [];
            rc.train_reservoirReadoutWashed = [];
            rc.train_reservoirTarget = cell(rc.sizeoutput,1);
            rc.train_reservoirTargetWashed = cell(rc.sizeoutput,1);
            for i = 1:rc.sizeoutput
                rc.train_reservoirTargetWashed{i} = [];
            end
            rc.predict_internalState = [];
            rc.predict_reservoirReadout = [];
            rc.xnorm = ones(rc.sizeinput,1);
        end

        function [x, target] = traintest(rc, simname)

            [const, x, target, internalState, ~] = rc.runsim(simname);

            v1 = x(1:end/2,:);
            if rc.ifxnorm
                % rc.xnorm = median(abs(v1),2);
                rc.xnorm = max(abs(v1),[],2)/2;
                rc.xnorm = [rc.xnorm; rc.xnorm];
            end
        end

        function [traindata] = traindatacollect(rc, simname, washout)

            [const, x, target, internalState, out] = rc.runsim(simname);
            
            rc.train_internalState = internalState;

            rc.train_reservoirReadout = [const'; x; rc.train_internalState'];
            rc.train_reservoirReadoutWashed = [rc.train_reservoirReadoutWashed rc.train_reservoirReadout(:,washout+1:end)];
            
            for i = 1:rc.sizeoutput
                rc.train_reservoirTarget{i} = target{i};
                rc.train_reservoirTargetWashed{i} = [rc.train_reservoirTargetWashed{i}; rc.train_reservoirTarget{i}(washout+1:end)];
            end

            outputoriginal = squeeze(out.yout{3}.Values.outputoriginal.Data);
            outputselected = squeeze(out.yout{3}.Values.outputselected.Data)';

            x = squeeze(out.yout{1}.Values.x.Data);
            x_lost = squeeze(out.yout{1}.Values.x_lost.Data);
            act = outputselected(:,1);
            act_real = squeeze(out.yout{4}.Values.actuationtorque.Data);

            allstate = [];
            try
                c = struct2cell(out.yout{2}.Values);
                for i = 1:size(c,1)
                    allstate(:,i*2-1) = squeeze(c{i,1}.theta.Data);
                    allstate(:,i*2) = squeeze(c{i,1}.thetadot.Data);
                end
            catch
                disp('no res state');
            end

            traindata = [const'; x_lost; internalState'; outputoriginal'; outputselected'; x; x_lost; act'; act_real'; allstate'; target{1}'];
            

        end

        function [train_output, train_target] = train(rc, varargin)
            
            planning  = false;

            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    case 'planning', planning = varargin{i+1};
                    otherwise, error('the option does not exist');
                end
            end

            if planning == 1
                train_output = cell(rc.sizeoutput,1);
                train_target = cell(rc.sizeoutput,1); 
                rc.Woutmat = [];
    
                for i = 1:rc.sizeoutput
                    if i == 1
                        X = rc.train_reservoirReadoutWashed;
                        Y = rc.train_reservoirTargetWashed{i};
            
                        rc.Wout{i} = Y'*X'*inv(X*X'+rc.lambda*eye(size(X,1))); 
                        rc.Woutmat = [rc.Woutmat; rc.Wout{i}];
                    else
                        X = rc.train_reservoirReadoutWashed([1:3 6:end],:); 
                        Y = rc.train_reservoirTargetWashed{i};
            
                        rc.Wout{i} = Y'*X'*inv(X*X'+rc.lambda*eye(size(X,1))); 
                        rc.Wout{i} = [rc.Wout{i}(1:3) 0 0 rc.Wout{i}(4:end)];
                        rc.Woutmat = [rc.Woutmat; rc.Wout{i}];
                    end
        
                    train_output{i} = rc.Wout{i}*rc.train_reservoirReadout;
                    train_output{i} = train_output{i}';
                    train_target{i} = rc.train_reservoirTarget{i};
                end
            else
                train_output = cell(rc.sizeoutput,1);
                train_target = cell(rc.sizeoutput,1);
                rc.Woutmat = [];
    
                for i = 1:rc.sizeoutput
                    X = rc.train_reservoirReadoutWashed;
                    Y = rc.train_reservoirTargetWashed{i};
        
                    rc.Wout{i} = Y'*X'*inv(X*X'+rc.lambda*eye(size(X,1))); 
                    rc.Woutmat = [rc.Woutmat; rc.Wout{i}];
        
                    train_output{i} = rc.Wout{i}*rc.train_reservoirReadout;
                    train_output{i} = train_output{i}';
                    train_target{i} = rc.train_reservoirTarget{i};
                end
            end

        end

        function [predict_output, predict_target] = predict(rc, simname)

            predict_output = cell(rc.sizeoutput,1);
            predict_target = cell(rc.sizeoutput,1);
        
            [const, x, target, internalState, ~] = rc.runsim(simname);

            rc.predict_internalState = internalState;

            rc.predict_reservoirReadout = [const'; x; rc.predict_internalState'];
            
            for i = 1:rc.sizeoutput
                predict_target{i} = target{i};
                predict_output{i} = rc.Wout{i}*rc.predict_reservoirReadout;
                predict_output{i} = predict_output{i}';
            end

        end

        function [controloutput] = robotcontrol(rc, simname)

            [const, x, target, internalState, out] = rc.runsim(simname);

            outputoriginal = squeeze(out.yout{3}.Values.outputoriginal.Data);
            outputselected = squeeze(out.yout{3}.Values.outputselected.Data)';

            x = squeeze(out.yout{1}.Values.x.Data);
            x_lost = squeeze(out.yout{1}.Values.x_lost.Data);
            act = outputselected(:,1);
            act_real = squeeze(out.yout{4}.Values.actuationtorque.Data);

            allstate = [];
            try
                c = struct2cell(out.yout{2}.Values);
                for i = 1:size(c,1)
                    allstate(:,i*2-1) = squeeze(c{i,1}.theta.Data);
                    allstate(:,i*2) = squeeze(c{i,1}.thetadot.Data);
                end
            catch
                disp('no res state');
            end

            controloutput = [const'; x_lost; internalState'; outputoriginal'; outputselected'; x; x_lost; act'; act_real'; allstate'; target{1}'];
            
        end

        function [const, x, target, internalState, out] = runsim(rc, simname)
            out = sim(simname, "TimeOut", 60);

            switch simname
                case {"PR_sim.slx"}
                    const = squeeze(out.yout{1}.Values.const.Data);
                    x = squeeze(out.yout{1}.Values.selectedinput.Data)';
                    target = cell(rc.sizeoutput,1);
                    c = struct2cell(out.yout{1}.Values.target);
                    for i = 1:rc.sizeoutput
                        target{i} = squeeze(c{i,1}.Data);
                    end
                    c = struct2cell(out.yout{2}.Values);
                    for i = 1:4
                        internalState(:,i) = squeeze(c{i,1}.thetasin.Data);
                    end
                case {"PR_planningsim.slx"}
                    const = squeeze(out.yout{1}.Values.const.Data);
                    x = squeeze(out.yout{1}.Values.x.Data);
                    target = cell(rc.sizeoutput,1);
                    tg = squeeze(out.yout{1}.Values.target.Data);
                    target{1} = tg(1,:)';
                    target{2} = tg(2,:)';
                    target{3} = tg(3,:)';
                    c = struct2cell(out.yout{2}.Values);
                    idx = [2 3 5 6];
                    for i = 1:4
                        internalState(:,i) = squeeze(c{idx(i),1}.theta.Data);
                    end
                case {"PR_planningsim_2joint.slx"}
                    const = squeeze(out.yout{1}.Values.const.Data);
                    x = squeeze(out.yout{1}.Values.x.Data);
                    target = cell(rc.sizeoutput,1);
                    tg = squeeze(out.yout{1}.Values.target.Data);
                    target{1} = tg(1,:)';
                    target{2} = tg(2,:)';
                    target{3} = tg(3,:)';
                    c = struct2cell(out.yout{2}.Values);
                    % internalState = [];
                    idx = [2 4];
                    for i = 1:2
                        internalState(:,i) = squeeze(c{idx(i),1}.theta.Data);
                        % internalState(:,i) = 0*squeeze(c{idx(i),1}.theta.Data); %noreservoir
                    end
                otherwise, error('the simulation does not exist');
            end
        end

    end
end