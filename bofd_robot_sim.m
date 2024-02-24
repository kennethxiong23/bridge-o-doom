beta = 0.2;
points = linspace(0, 3.2, 1000);
x = 0.3960 * cos(2.65*(1.4 + points));
y = -0.99 * sin(1.4 + points);

% Define symbols
syms u t

% Parametric equations with beta * t
u = beta * t;
r_i = 0.3960 * cos(2.65*(1.4 + u));
r_j = -0.99 * sin(1.4 + u);
r_k = 0 * u;

% Derivatives and unit vectors
assume(t, 'real')
r = [r_i, r_j, r_k];
drdu = diff(r, t);
T_hat = simplify(drdu ./ norm(drdu));
dT_hatdu = diff(T_hat, t);
N_hat = simplify(dT_hatdu / norm(dT_hatdu));

% Connect to your Neato or the Simulator
[sensors, vels] = neatoSim(); % For simulator
% [sensors, vels] = neato('192.168.16.93'); % For physical Neato

fig = gcf;
% disp('Press enter to continue');
% pause;

% Timer for motion
drivetime = 7.7; % Duration of the drive
tic
while toc < drivetime
    t = toc; % Update time
    
    % Recalculate velocities based on the updated time 't'
    vl = calculateVl(t, beta);
    vr = calculateVr(t, beta);
    
    vels.lrWheelVelocitiesInMetersPerSecond = [vl, vr];
    
    pause(0.1); % Throttle update rate
end

% Stop the robot after the drive time
vels.lrWheelVelocitiesInMetersPerSecond = [0, 0];
pause(1);
close(fig);


function vl = calculateVl(t, beta)
    % Define the variables used in the omega expression for clarity
    u = beta * t;
    sigma_5 = (53 * u) / 20 + 371/100;
    sigma_3 = sin(u + 7/5);
    sigma_4 = cos(u + 7/5);
    sigma_1 = (1459143477 * cos(sigma_5) * sin(sigma_5) / 250000000) - (9801 * sigma_4 * sigma_3 / 5000);
    sigma_2 = (9801 * sigma_4^2 / 10000) + (27531009 * sin(sigma_5)^2 / 25000000);
    
    % Linear velocity V_n
    V_n = (99 * sqrt(2809 * beta^2 * sin((53 * beta * t) / 20 + 371/100)^2 + 2500 * beta^2 * cos(beta * t + 7/5)^2)) / 5000;
    
    % Angular velocity omega
    omega = sqrt(((278091 * cos(sigma_5) / (100000 * sqrt(sigma_2)) - (5247 * sin(sigma_5) * sigma_1) / (10000 * sigma_2^(3/2)))^2 + ((99 * sigma_3 / (100 * sqrt(sigma_2)) + (99 * sigma_4 * sigma_1) / (200 * sigma_2^(3/2)))^2)) * sqrt((27531009 * beta^2 * sin((53 * beta * t) / 20 + 371/100)^2) / 25000000 + (9801 * beta^2 * cos(beta * t + 7/5)^2) / 10000));
    
    % Calculate left wheel velocity
    vl = V_n - omega * 0.245 / 2;

    % Limit the right wheel velocity to 0.3 meters/second
    if abs(vl) > 0.3
        vl = sign(vl) * 0.3;
    end
end

function vr = calculateVr(t, beta)
    % Define the variables used in the omega expression for clarity
    u = beta * t;
    sigma_5 = (53 * u) / 20 + 371/100;
    sigma_3 = sin(u + 7/5);
    sigma_4 = cos(u + 7/5);
    sigma_1 = (1459143477 * cos(sigma_5) * sin(sigma_5) / 250000000) - (9801 * sigma_4 * sigma_3 / 5000);
    sigma_2 = (9801 * sigma_4^2 / 10000) + (27531009 * sin(sigma_5)^2 / 25000000);
    
    % Linear velocity V_n
    V_n = (99 * sqrt(2809 * beta^2 * sin((53 * beta * t) / 20 + 371/100)^2 + 2500 * beta^2 * cos(beta * t + 7/5)^2)) / 5000;
    
    % Angular velocity omega
    omega = sqrt(((278091 * cos(sigma_5) / (100000 * sqrt(sigma_2)) - (5247 * sin(sigma_5) * sigma_1) / (10000 * sigma_2^(3/2)))^2 + ((99 * sigma_3 / (100 * sqrt(sigma_2)) + (99 * sigma_4 * sigma_1) / (200 * sigma_2^(3/2)))^2)) * sqrt((27531009 * beta^2 * sin((53 * beta * t) / 20 + 371/100)^2) / 25000000 + (9801 * beta^2 * cos(beta * t + 7/5)^2) / 10000));
    
    % Calculate right wheel velocity
    vr = V_n + omega * 0.245 / 2;

    % Limit the right wheel velocity to 0.3 meters/second
    if abs(vr) > 0.3
        vr = sign(vr) * 0.3;
    end
end
