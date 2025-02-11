%% Setup robot
timestep = .01
travelTime = 20; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(2); % Write travel time




%% Program 
% % % % clc; clear;
import lspb.m.*;
% % % % import FKinSpace.m.*;
% % % % import IKinSpace.m.*;
% % % % warning('off', 'all')
% % % % % Task Space
% % % % S0 = [0.185, -0.185, 0.185];
% % % % SM = [0.185, 0, 0.240];
% % % % SF = [0.185, 0.170, 0.070];
% % % % Step = 200;
% % % % 
% % % % [Sxfst, SDxfst, SDDxfst] = lspb(S0(1), SM(1), Step/2);
% % % % [Syfst, SDyfst, SDDyfst] = lspb(S0(2), SM(2), Step/2);
% % % % [Szfst, SDzfst, SDDzfst] = lspb(S0(3), SM(3), Step/2);
% % % % 
% % % % Sfst = [Sxfst, Syfst, Szfst];
% % % % SDfst = [SDxfst, SDyfst, SDzfst];
% % % % SDDfst = [SDDxfst, SDDyfst, SDDzfst];
% % % % 
% % % % [Sxsnd, SDxsnd, SDDxsnd] = lspb(SM(1), SF(1), Step/2);
% % % % [Sysnd, SDysnd, SDDysnd] = lspb(SM(2), SF(2), Step/2);
% % % % [Szsnd, SDzsnd, SDDzsnd] = lspb(SM(3), SF(3), Step/2);
% % % % 
% % % % Ssnd = [Sxsnd, Sysnd, Szsnd];
% % % % SDsnd = [SDxsnd, SDysnd, SDzsnd];
% % % % SDDsnd = [SDDxsnd, SDDysnd, SDDzsnd];
% % % % 
% % % % S1 = [Sfst; Ssnd];
% % % % SD1 = [SDfst, SDsnd];
% % % % SDD1 = [SDDfst, SDDsnd];
% % % % 
% % % % 
% % % % 
% % % % % Joint Space - Single
% % % % % S0 = 0;
% % % % % SF = 45;
% % % % % M = 10;
% % % % % 
% % % % % [S, SD, SDD] = lspb(S0, SF, M);
% % % % 
% % % % 
% % % % 
% % % % % Joint Space - Multi
% % % % 
% Values for Waypoints A C B in degrees
SX = readings(1,1:4);
S0 = [-45, -11.1994, 30.6169, -19.4176];
SM = [0, -47.6140, 28.8178, 18.7962];
SF = [42.5805, 31.4419, 50.2107, -81.6526];
% % % % 
% % % % % Step = 20;
% % % % % 
% % % % % [S1fst, SD1fst, SDD1fst] = lspb(S0(1), SM(1), Step/2);
% % % % % [S2fst, SD2fst, SDD2fst] = lspb(S0(2), SM(2), Step/2);
% % % % % [S3fst, SD3fst, SDD3fst] = lspb(S0(3), SM(3), Step/2);
% % % % % [S4fst, SD4fst, SDD4fst] = lspb(S0(4), SM(4), Step/2);
% % % % % 
% % % % % [S1snd, SD1snd, SDD1snd] = lspb(SM(1), SF(1), Step/2);
% % % % % [S2snd, SD2snd, SDD2snd] = lspb(SM(2), SF(2), Step/2);
% % % % % [S3snd, SD3snd, SDD3snd] = lspb(SM(3), SF(3), Step/2);
% % % % % [S4snd, SD4snd, SDD4snd] = lspb(SM(4), SF(4), Step/2);
% % % % % 
% % % % % S1 = [S1fst; S1snd];
% % % % % S2 = [S2fst; S2snd];
% % % % % S3 = [S3fst; S3snd];
% % % % % S4 = [S4fst; S4snd];
% % % % % 
% % % % % SD1 = [SD1fst; SD1snd];
% % % % % SD2 = [SD2fst; SD2snd];
% % % % % SD3 = [SD3fst; SD3snd];
% % % % % SD4 = [SD4fst; SD4snd];
% % % % % 
% % % % % SDD1 = [SDD1fst; SDD1snd];
% % % % % SDD2 = [SDD2fst; SDD2snd];
% % % % % SDD3 = [SDD3fst; SDD3snd];
% % % % % SDD4 = [SDD4fst; SDD4snd];
% % % % % 
% % % % % S = [S1, S2, S3, S4];
% % % % % SD = [SD1, SD2, SD3, SD4];
% % % % % SDD = [SDD1, SDD2, SDD3, SDD4];
% % % % 
% % % % 
% % % % 
Slist = [[0; 0; 1; 0; 0; 0], ...
       [0; 1; 0; -0.096; 0; 0], ...
       [0; 1; 0; -0.224; 0; 0.024], ...
       [0; 1; 0; -0.224; 0; 0.148]];
M = [[1, 0, 0, 0.294]; [0, 1, 0, 0]; [0, 0, 1, 0.224]; [0, 0, 0, 1]];
% % % % 
% % % % tx = zeros(Step, 1);
% % % % ty = zeros(Step, 1);
% % % % tz = zeros(Step, 1);
% % % % joinpos = zeros(4, 1, Step);
% % % % thetalist0 = [deg2rad(-45); 0; 0; 0];
% % % % for i = 1:length(S1)
% % % %     % Joint Space
% % % %     % thetalist = deg2rad([S1(i); S2(i); S3(i); S4(i)]);
% % % %     % tempFK = FKinSpace(M, Slist, thetalist);
% % % % 
% % % %     % Task Space
% % % %     tx(i) = S1(i, 1);
% % % %     ty(i) = S1(i, 2);
% % % %     tz(i) = S1(i, 3);
% % % % 
% % % %     T = eye(4);
% % % %     T(1:3, 4) = S1(i, 1:3);
% % % %     a = atan2(ty(i), tx(i));
% % % %     T(1:3, 1:3) = eul2rotm([a, 0, 0]);
% % % %     eomg = 0.01;
% % % %     ev = 0.001;
% % % %     [tempIK, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);
% % % %     jointpos(:, :, i) = tempIK;
% % % %     thetalist0 = tempIK;
% % % %     display(rad2deg(thetalist0))
% % % %     if (success)
% % % %         tempFK = FKinSpace(M, Slist, tempIK);
% % % % 
% % % %     tx(i) = tempFK(1, 4);
% % % %     ty(i) = tempFK(2, 4);
% % % %     tz(i) = tempFK(3, 4);
% % % %     else
% % % %     disp("adf")
% % % % 
% % % %     end
% % % % 
% % % % 
% % % % 
% % % % end
% robot.writeJoints([0, 0, -20, 0]);
%robot.writeJoints(S0); % Write joints to zero position=

%% Go to starting position
robot.writeMode('cp');
robot.writeJoints([0, 0, 0, 0]);
pause(2);
robot.writeJoints(S0);
pause(2);

robot.writeVelocities([0, 0, 0, 0]);
robot.writeMode('v');
robot.writeTime(timestep); % Write travel time
% sampleCount = travelTime/timestep/2;
% positions = zeros(sampleCount, 4);
% i = 1;
% tic
% display("Moving to A position")
% while toc < travelTime/2
% 
%         [S1fst, SD1fst, SDD1fst] = lspb(SX(1), S0(1), [0, toc, travelTime/2]);
%         [S2fst, SD2fst, SDD2fst] = lspb(SX(2), S0(2), [0, toc, travelTime/2]);
%         [S3fst, SD3fst, SDD3fst] = lspb(SX(3), S0(3), [0, toc, travelTime/2]);
%         [S4fst, SD4fst, SDD4fst] = lspb(SX(4), S0(4), [0, toc, travelTime/2]);
%         %display(SD1fst(2))
%         %display([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]/timestep)
%         robot.writeVelocities([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]);
%         positions(i, 1:4) = [SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)];
% 
%         pause(timestep)
%         % disp(robot.getJointsReadings()); % Read joint values
%         % readings = robot.getJointsReadings()
%         % currents(i) = readings(3,2);
%         i = i+1;
% end
% robot.writeVelocities([0, 0, 0, 0]);
% figure
% plot(positions)
% title('Joint Velocity Over Time')
% legend('Joint 1','Joint2', 'Joint3', 'Joint4')
% display("done")
% 
% pause(2)
% step = 2;
% i = 1;
% currents = zeros(1,step);
i = 1;
sampleCount = travelTime/timestep;
positions = zeros(sampleCount, 4);
% tic

display("moving from A to C")
%% Execution of Waypoint A to C
tic
maxBase = 0;
while toc < travelTime/4
        [S1fst, SD1fst, SDD1fst] = lspb(S0(1), SM(1), [0, toc, travelTime/2]);
        [S2fst, SD2fst, SDD2fst] = lspb(S0(2), SM(2), [0, toc, travelTime/2]);
        [S3fst, SD3fst, SDD3fst] = lspb(S0(3), SM(3), [0, toc, travelTime/2]);
        [S4fst, SD4fst, SDD4fst] = lspb(S0(4), SM(4), [0, toc, travelTime/2]);
        %display([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]/timestep)

        robot.writeVelocities([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]*0.8);
        positions(i, 1:4) = [SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]*0.8;
        i = i+1;
        pause(timestep)
        % disp(robot.getJointsReadings()); % Read joint values
        % readings = robot.getJointsReadings()
        % currents(i) = readings(3,2);
        % i = i+1;
end

        [S1fst, o1, SDD1fst] = lspb(S0(1), SM(1), [0, travelTime/4, travelTime/2]);
        [S2fst, o2, SDD2fst] = lspb(S0(2), SM(2), [0, travelTime/4, travelTime/2]);
        [S3fst, o3, SDD3fst] = lspb(S0(3), SM(3), [0, travelTime/4, travelTime/2]);
        [S4fst, o4, SDD4fst] = lspb(S0(4), SM(4), [0, travelTime/4, travelTime/2]);

        [S1fst, d1, SDD1fst] = lspb(SM(1), SF(1), [0, travelTime/4, travelTime/2]);
        [S2fst, d2, SDD2fst] = lspb(SM(2), SF(2), [0, travelTime/4, travelTime/2]);
        [S3fst, d3, SDD3fst] = lspb(SM(3), SF(3), [0, travelTime/4, travelTime/2]);
        [S4fst, d4, SDD4fst] = lspb(SM(4), SF(4), [0, travelTime/4, travelTime/2]);

        m1 = (d1(2)-o1(2))/(travelTime/2);
        m2 = (d2(2)-o2(2))/(travelTime/2);
        m3 = (d3(2)-o3(2))/(travelTime/2);
        m4 = (d4(2)-o4(2))/(travelTime/2);
        
        
        
flag = true;
while toc < travelTime * (3/4)
    if toc > travelTime/2 && flag
        flag = false;
        display("Moving from C to B")
    end
    %vels = [toc-(travelTime/4)*m1, toc-(travelTime/4)*m2, toc-(travelTime/4)*m3, toc-(travelTime/4)*m4]
    vels = (toc-travelTime/4) * [m1, m2, m3, m4] + [o1(2), o2(2), o3(2), o4(2)];
    positions(i, 1:4) = vels*0.8;
    robot.writeVelocities(vels*0.8);
    pause(timestep)
    i = i+1;
end

%% Execution of Waypoint C to B

while toc < travelTime
        [S1fst, SD1fst, SDD1fst] = lspb(SM(1), SF(1), [0, toc-travelTime/2, travelTime/4, travelTime/2]);
        [S2fst, SD2fst, SDD2fst] = lspb(SM(2), SF(2), [0, toc-travelTime/2, travelTime/2]);
        [S3fst, SD3fst, SDD3fst] = lspb(SM(3), SF(3), [0, toc-travelTime/2, travelTime/2]);
        [S4fst, SD4fst, SDD4fst] = lspb(SM(4), SF(4), [0, toc-travelTime/2, travelTime/2]);
        % display([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]/timestep)

        robot.writeVelocities([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]*0.8); 
        positions(i, 1:4) = [SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]*0.8;
        i = i+1;
        pause(timestep)
        % disp(robot.getJointsReadings()); % Read joint values
        % readings = robot.getJointsReadings()
        % currents(i) = readings(3,2);
        % i = i+1;
    end
robot.writeVelocities([0, 0, 0, 0]);
robot.writeGripper(false);

display('Waypoint A to C to B movement finished');

%Generating the plot of joint velocity
figure
plot(positions)
title('Joint Velocity Over Time')
legend('Joint 1','Joint2', 'Joint3', 'Joint4')

pause(1);