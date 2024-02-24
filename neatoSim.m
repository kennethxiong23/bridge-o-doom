function [neatoSensors, neatoVelocities] = neatoSim(varargin)
%enter the pose as x,y,theta
neatoSensors = NeatoSensors();
neatoVelocities = NeatoVelocities();
if nargin>1
    x=varargin{1};
    y=varargin{2};
    theta=varargin{3};
else
    x=0.0;
    y=0.0;
    theta=0.0;
end
if nargin < 4
    env=0;
else
    env=varargin{4};
end
boundary=5;
%%Define flatland environment
[x1,y1] = meshgrid(linspace(-boundary,boundary,100),linspace(-boundary,boundary,100));
flatland = 16*exp(-x1.^2/2 - y1.^2/2 - x1.*y1/2) + 4*exp(-(x1+1.5).^2-(y1+2.5).^2);

% x, y, theta, vL, vR, encoderL, encoderR
robotState = [x; y; theta; 0.0; 0.0; 0.0; 0.0];
timeStamp = [];     % set to empty until we've gone through loop once
deltaT = [];
robotWidth = 0.35;   % just for visualization purposes (didn't actually measure)
% This script provides a visualization and teleop interface to the Neato
% robots.  In order to use it, you first must connect to the Neatos
% using the procedure on the webpage.  To launch the application run:
%
%    neatoSim()
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
    fTopDown = figure;
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
    BASE_WIDTH = 245;    % millimeters
    MAX_SPEED = 300;     % millimeters/second
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
        try
            if ~isempty(timeStamp)
                deltaT = toc(timeStamp);
                robotState = forwardKinematics(robotState, deltaT);
            end
            timeStamp = tic;
            if length(neatoVelocities.lrWheelVelocitiesInMetersPerSecond) == 2
                setMotors(1000*neatoVelocities.lrWheelVelocitiesInMetersPerSecond(1), ...
                    1000*neatoVelocities.lrWheelVelocitiesInMetersPerSecond(2), ...
                    1000*max(abs(neatoVelocities.lrWheelVelocitiesInMetersPerSecond)));
            end

            % generate these
            accelAsDoubles = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0];    % only model acceleration due to gravity
            motorsAsDoubles = [0.0, 0.0];           % not yet supported
            digitalAsDoubles = zeros(1, 4);         % not yet supported
            rangesAsDoubles = zeros(1, 360);        % not yet supported

            % copy the values from the UDP packet into the shared struct
            neatoSensors.bumpers = digitalAsDoubles;
            neatoSensors.ranges = rangesAsDoubles;
            neatoSensors.thetasInRadians = deg2rad(0:359);
            neatoSensors.accels = accelAsDoubles;
            neatoSensors.encoders = [robotState(6) robotState(7)];

            set(0, 'CurrentFigure', fTopDown);
            P = drawRobotBody(robotState);
            set(gca,'Nextplot','ReplaceChildren');
            if env==1
                contour(x1, y1, flatland,'LineWidth',3)
                hold on
            end   
            plot(P);
            hold on;
            quiver(robotState(1), robotState(2), cos(robotState(3)), sin(robotState(3)), 'color', 'r', 'maxheadsize', 2, 'linewidth', 1);
            xlim([-boundary boundary]);
            ylim([-boundary boundary]);
            axis equal;
            grid on;
            prev = gcf;
            set(0,'CurrentFigure',f);
            set(gca,'Nextplot','ReplaceChildren');
            polarplot(deg2rad(0:359), rangesAsDoubles, 'b.');
            rlim([0 5]);
            hold off;
            if isvalid(prev)
                set(0, 'CurrentFigure', prev);
            end
        catch ex
            % ex
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
        % update speeds here
        robotState(4) = l/1000.0;
        robotState(5) = r/1000.0;
    end

    function newState = forwardKinematics(state, deltaT)
        % https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
        l = BASE_WIDTH/1000.0;
        vF = (state(4) + state(5))/2.0;
        omega = (state(5) - state(4))/l;
        newState = state;
        R = (state(4) + state(5))/(state(5) - state(4))*l/2.0;
        if isfinite(R)
            ICC = [state(1) - R*sin(state(3)) state(2) + R*cos(state(3))];
            newState(1:3) = [cos(omega*deltaT) -sin(omega*deltaT) 0;...
                             sin(omega*deltaT) cos(omega*deltaT) 0;...
                             0 0 1]*[state(1) - ICC(1);...
                                     state(2) - ICC(2);...
                                     state(3)] + [ICC(1);...
                                                  ICC(2);...
                                                  omega*deltaT];

        else
            newState(1:2) = [cos(state(3));...
                             sin(state(3))]*deltaT*vF + [state(1);...
                                                        state(2)];
        end
        newState(6) = newState(6) + state(4)*deltaT;
        newState(7) = newState(7) + state(5)*deltaT;
    end

    function P = drawRobotBody(robotState)
        P = polyshape([robotState(1)-robotWidth/2, robotState(2) - robotWidth/2;...
               robotState(1)-robotWidth/2, robotState(2) + robotWidth/2;...
               robotState(1)+robotWidth/2, robotState(2) + robotWidth/2;...
               robotState(1)+robotWidth/2, robotState(2) - robotWidth/2]);
        P = rotate(P, rad2deg(robotState(3)), robotState(1:2)');
    end
end