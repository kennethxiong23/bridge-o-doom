function myRobotProgram(sensors, vels)
while true
    % this first line is just for showing sensor as text
    sensors.ranges(10)
    if max(sensors.bumpers) > 0 || ...
        (sensors.ranges(10) ~= 0 && sensors.ranges(10) < 0.1)
        break;
    end
    vels.lrWheelVelocitiesInMetersPerSecond = [0.1, 0.1];
    pause(0.05);    
end
vels.lrWheelVelocitiesInMetersPerSecond = [0.0, 0.0];
end