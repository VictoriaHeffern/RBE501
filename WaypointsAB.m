%% Setup robot
timestep = .1
travelTime = 20; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(2); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Program 
% % % % clc; clear;
import lspb.m.*;
robot.writeMode('cp');

% Values for Waypoints A & B in degrees
S0 = [-45, -11.1994, 30.6169, -19.4176];
SF = [42.5805, 31.4419, 50.2107, -81.6526];

% % % % 
Slist = [[0; 0; 1; 0; 0; 0], ...
       [0; 1; 0; -0.096; 0; 0], ...
       [0; 1; 0; -0.224; 0; 0.024], ...
       [0; 1; 0; -0.224; 0; 0.148]];
M = [[1, 0, 0, 0.294]; [0, 1, 0, 0]; [0, 0, 1, 0.224]; [0, 0, 0, 1]];

robot.writeJoints([0, 0, 0, 0]); % return to home configuration

pause(2)
step = 2;
i = 1;
currents = zeros(1,step);
sampleCount = travelTime/timestep;
positions = zeros(sampleCount, 4);
robot.writeJoints(S0);
pause(3)
robot.writeTime(timestep);
tic


%% Execution of Waypoint A to B
while toc < travelTime
        [S1fst, SD1fst, SDD1fst] = lspb(S0(1), SF(1), [0, toc, travelTime]);
        [S2fst, SD2fst, SDD2fst] = lspb(S0(2), SF(2), [0, toc, travelTime]);
        [S3fst, SD3fst, SDD3fst] = lspb(S0(3), SF(3), [0, toc, travelTime]);
        [S4fst, SD4fst, SDD4fst] = lspb(S0(4), SF(4), [0, toc, travelTime]);
        display([S1fst(2), S2fst(2), S3fst(2), S4fst(2)])
        robot.writeJoints([S1fst(2), S2fst(2), S3fst(2), S4fst(2)]);
        positions(i, 1:4) = [S1fst(2), S2fst(2), S3fst(2), S4fst(2)];
        i = i+1;
        pause(timestep*.9)
        % disp(robot.getJointsReadings()); % Read joint values
        % readings = robot.getJointsReadings()
        % currents(i) = readings(3,2);
        % i = i+1;
end

display('Waypoint A to B movement finished');

robot.writeGripper(false);

%Generating the plot of joint angles
figure
plot(positions)
title('Joint Angle Over Time')
legend('Joint 1','Joint2', 'Joint3', 'Joint4')

pause(1);