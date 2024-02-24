function [neatoSensors, neatoVelocities] = neato(ip,varargin)
if nargin>1
    port_num=varargin{1};
else
    port_num=7921;
end
neatoSensors = NeatoSensors();
neatoVelocities = NeatoVelocities();
% This script provides a visualization and teleop interface to the Neato
% robots.  In order to connect to the Neatos run:
%
%    neato('ipaddress')
%
% Running this script will startup a figure window that will show the 
% heading of the robot along with the laser scan data in the odometry
% coordinate system of the robot.
%
% To control the robot with the keyboard, you must have the focus on the
% visualizer window (i.e. click on the window).  The key mappings are:
%     i : forward
%     k : stop
%     j : left
%     l : right
%     , : backward
%     u : forward while turning left
%     o : forward while turning right
%     m : backward while turning left
%     . : backward while turning right
%
% Additionally there are sliders that control the both the forward and
% angular speed of the robot.
%
% To stop execution of the program, simply close the figure window.
    function setLinearVelocity(hObject, eventdata, handles)
        % callback function for the linear velocity slider
        v = get(hObject, 'Value');
    end

    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        if exist('T','var')
            stop(T);
        end
        if exist('sock','var')
            clear sock;
        end
        delete(gcf);
    end

    function setAngularVelocity(hObject, eventdata, handles)
        % callback function for the angular velocity slider
        w = get(hObject, 'Value');
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'i'
                linear = v;
                angular = 0;
            case 'u'
                linear = v;
                angular = w;
            case 'j'
                linear = 0;
                angular = w;
            case 'm'
                linear = -v;
                angular = -w;
            case 'comma'
                linear = -v;
                angular = 0;
            case 'period'
                linear = -v;
                angular = w;
            case 'l'
                linear = 0;
                angular = -w;
            case 'o'
                linear = v;
                angular = -w;
            otherwise
                linear = 0;
                angular = 0;
        end
        cmdVel(linear, angular);
    end
    last_cmd = [];
    v = 0.3;
    w = 0.8;
    stop_state = false;
	f = figure('CloseRequestFcn',@myCloseRequest);
    set(gca,'position',[.05,.20,.9,.7]);

    sld = uicontrol('Style', 'slider',...
        'Min',0,'Max',0.3,'Value',0.3,...
        'Position', [200 20 120 20],...
        'Callback', @setLinearVelocity);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[200 45 120 20],...
        'String','Linear Velocity Scale');
    sld = uicontrol('Style', 'slider',...
        'Min',0,'Max',0.6/.248,'Value',w,...
        'Position', [20 20 120 20],...
        'Callback', @setAngularVelocity);
    % Add a text uicontrol to label the slider.
    txt = uicontrol('Style','text',...
        'Position',[20 45 120 20],...
        'String','Angular Velocity Scale');

    set(f,'WindowKeyPressFcn', @keyPressedFunction);
    BASE_WIDTH = 248;    % millimeters
    MAX_SPEED = 300;     % millimeters/second
    t = tic;
    lowBattery = tic;
    sock = tcpclient(ip, 7777);
    pause(1);
    txt2=sprintf('protocolpreference True %d False',port_num);
    writeline(sock, txt2);
    pause(1);
    writeline(sock, 'testmode on');
    pause(1);
    writeline(sock, 'setldsrotation on');
    T = timer('Period',0.2,... %period
        'ExecutionMode','fixedRate',... %{singleShot,fixedRate,fixedSpacing,fixedDelay}
        'BusyMode','drop',... %{drop, error, queue}
        'TasksToExecute',inf,...          
        'StartDelay',1,...
        'TimerFcn',@(src,evt)socketLoop(),...
        'StartFcn',[],...
        'StopFcn',[],...
        'ErrorFcn',[]);
    start(T);

    function socketLoop()
        elapsedSinceKeepAlive = toc(t);
        if elapsedSinceKeepAlive > 10.0
            t = tic;
            writeline(sock, 'keepalive');
        end
        try
            if length(neatoVelocities.lrWheelVelocitiesInMetersPerSecond) == 2
                setMotors(round(1000*neatoVelocities.lrWheelVelocitiesInMetersPerSecond(1)), ...
                    round(1000*neatoVelocities.lrWheelVelocitiesInMetersPerSecond(2)), ...
                    round(1000*max(abs(neatoVelocities.lrWheelVelocitiesInMetersPerSecond))));
            end
            packet = judp('receive', port_num, 1600);
            accelAsDoubles = double(typecast(packet(1:24), 'single')');
            motorsAsDoubles = typecast(packet(25:40),'double')'/1000.0;
            digitalAsDoubles = typecast(packet(41:72),'double')';
            rangesAsDoubles = double(typecast(packet(73:end-4),'uint16'))'/1000.0;
            voltageAsDouble = double(typecast(packet(end-3:end-2), 'uint16'))/1000.0;
            fuelPercentAsDouble = double(typecast(packet(end-1:end), 'uint16'));
            if fuelPercentAsDouble < 30
                timeSinceLastBatteryWarning = toc(lowBattery);
                if timeSinceLastBatteryWarning > 2.0
                    lowBattery = tic;
                    beep();
                end
            end

            % copy the values from the UDP packet into the shared struct
            neatoSensors.bumpers = digitalAsDoubles;
            neatoSensors.ranges = rangesAsDoubles;
            neatoSensors.thetasInRadians = deg2rad(0:359);
            neatoSensors.accels = accelAsDoubles;
            neatoSensors.encoders = motorsAsDoubles;
            neatoSensors.batteryVoltage = voltageAsDouble;
            neatoSensors.fuelPercent = fuelPercentAsDouble;
            
            prev = gcf;
            resendLastMotorCommand();
            set(0,'CurrentFigure',f);
            set(gca,'Nextplot','ReplaceChildren');
            polarplot(deg2rad(0:359), rangesAsDoubles, 'b.');
            rlim([0 5]);
            hold off;
            if isvalid(prev)
                set(0, 'CurrentFigure', prev);
            end
        catch ex
            %ex
        end
        drawnow;
    end
    function cmdVel(linear, angular)
        x = linear * 1000;
        th = angular * (BASE_WIDTH/2);
        k = max(abs(x-th),abs(x+th));
        if k > MAX_SPEED
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k;
        end
        cmd_vel = [ round(x-th) , round(x+th) ];
        neatoVelocities.lrWheelVelocitiesInMetersPerSecond = cmd_vel/1000.0;
        setMotors(cmd_vel(1), cmd_vel(2), max(abs(cmd_vel)));
    end

    function setMotors(l, r, s)
        if isnan(l) || isnan(r) || isnan(s)
            warning('wheel velocities contain NaN values')
            return
        end
        if isinf(l) || isinf(r) || isinf(s)
            warning('wheel velocities contain Inf values')
            return
        end
        if abs(l/1000) > 0.3 || abs(r/1000.0) > 0.3
            warning('trying to set wheel speeds greater than 0.3 m/s')
            return
        end
        if l == 0.0 && r == 0.0 && s == 0.0
            if ~stop_state
                writeline(sock, 'setmotor 1 1 1');
                stop_state = true;
                last_cmd = [0.0, 0.0, 0.0];
            else
                writeline(sock, 'setmotor 0 0 0');
            end
        else
            stop_state = false;
            last_cmd = [l,r,s];
            writeline(sock, sprintf('setmotor %d %d %d', l, r, s));
        end
    end

    function resendLastMotorCommand()
        if ~isempty(last_cmd)
            setMotors(last_cmd(1), last_cmd(2), last_cmd(3));
        end
    end
end
