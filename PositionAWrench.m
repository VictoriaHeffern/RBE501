%% Setup robot
timestep = .1
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(timestep); % Write travel time
robot.writeMotorState(true); % Write position mode

%% Program 
% clc; clear;
import lspb.m.*;

S0 = [-45, -11.1994, 30.6169, -19.4176]; % Values for Waypoint A 

Slist =[[0; 0; 1; 0; 0; 0], ... % space twists
       [0; 1; 0; -0.096; 0; 0], ...
       [0; 1; 0; -0.224; 0; 0.024], ...
       [0; 1; 0; -0.224; 0; 0.148]];
M = [[1, 0, 0, 0.294]; [0, 1, 0, 0]; [0, 0, 1, 0.224]; [0, 0, 0, 1]]; % home 

Blist = [[0; 0; 1; 0; 0.25711; 0], ...
       [0; 1; 0; 0.12784; 0; -0.28111], ...
       [0; 1; 0; 0; 0; -0.25711], ...
       [0; 1; 0; 0; 0; -0.13341]];


robot.writeMode('cp'); %set the robot to current-based position mode
robot.writeTime(2) % set travel time to 2
robot.writeJoints([0, 0, 0, 0]);% move to home position

pause(2)
step = 2;
i = 1;
currents = zeros(1,step);
sampleCount = travelTime/timestep;
positions = zeros(sampleCount, 4);
tic

%% Moving to Waypoint A
robot.writeJoints(S0)
pause(5) % give time for the robot to settle before we take readings
readings = robot.getJointsReadings();
td2g = (readings(3,:)*1.769 -0.2214)/1000; % gets readings with gravity

bodyjacobian= JacobianBody(Blist, deg2rad(S0));
sampleCount = 100;
sampleRate = 0.1;
buckets = [0, 0, 0, 0, 0, 0]
tic
while (toc < sampleCount * sampleRate)
    readings = robot.getJointsReadings();
    torques = ((readings(3,:)*1.769 -0.2214)/1000); %removes the gravity component: td2g
    display(torques);

    % Finding the force at the tip
    % we do this by taking our taulist (torques) and then multiplying it by
    % the inverse of the body jacobian
    forcetip=torques*pinv(bodyjacobian);
    buckets = buckets + forcetip;
    display(forcetip);
    pause(sampleRate);
end
buckets = buckets/sampleCount;

while (true)
    readings = robot.getJointsReadings();

    % torque equation derived from the graph of torque v current
    torques = ((readings(3,:)*1.769 -0.2214)/1000); %removes the gravity component: td2g
    display(torques);

    % Finding the force at the tip
    % we do this by taking our taulist (torques) and then multiplying it by
    % the inverse of the body jacobian
    forcetip=(torques*pinv(bodyjacobian)) -buckets;

    display(forcetip);
end