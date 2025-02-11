%% Setup robot
timestep = .01
travelTime = 10; % Defines the travel time
robot = Robot(); % Creates robot object
robot.writeTime(2); % Write travel time

g = [0; 0; -9.8]; % gravity

Ftip = [0; 0; 0; 0; 0; 0]; % force generated at the tip

M01 = [[1, 0, 0, 0]; [0, 1, 0, 0.0006]; [0, 0, 1, 0.0781]; [0, 0, 0, 1]]; % Mlist
M12 = [[1, 0, 0, 0.00501]; [0, 1, 0, -0.00078]; [0, 0, 1, 0.12226]; [0, 0, 0, 1]];
M23 = [[1, 0, 0, 0.1127]; [0, 1, 0, 0.00019]; [0, 0, 1, 0.0246]; [0, 0, 0, 1]];
M34 = [[1, 0, 0, 0.09661]; [0, 1, 0, -0.00009]; [0, 0, 1, 0.00573]; [0, 0, 0, 1]];
M45 = [[1, 0, 0, 0.08035]; [0, 1, 0, 0.00007]; [0, 0, 1, -0.00209]; [0, 0, 0, 1]];

G1 = diag([0.00004321682, 0.00004321682, 0.00002761718, 0.114, 0.114, 0.114]);
G2 = diag([0.00022507475, 0.00022507475, 0.0000477574, 0.138, 0.138, 0.138]);
G3 = diag([0.00015081977, 0.00015081977, 0.00002729551, 0.118, 0.118, 0.118]);
G4 = diag([0.00028711671, 0.00028711671, 0.00018019833 0.218, 0.218, 0.218]);

Glist = cat(3, G1, G2, G3, G4);
Mlist = cat(3, M01, M12, M23, M34, M45); 

Slist =[[0; 0; 1; 0; 0; 0], ...
       [0; 1; 0; -0.096; 0; 0], ...
       [0; 1; 0; -0.224; 0; 0.024], ...
       [0; 1; 0; -0.224; 0; 0.148]];



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
positions = zeros(200, 4);
% tic

%% Execution of Waypoint A to C
tic
maxBase = 0;
while toc < travelTime
        [S1fst, SD1fst, SDD1fst] = lspb(S0(1), SF(1), [0, toc, travelTime]);
        [S2fst, SD2fst, SDD2fst] = lspb(S0(2), SF(2), [0, toc, travelTime]);
        [S3fst, SD3fst, SDD3fst] = lspb(S0(3), SF(3), [0, toc, travelTime]);
        [S4fst, SD4fst, SDD4fst] = lspb(S0(4), SF(4), [0, toc, travelTime]);
        %display([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]/timestep)

        robot.writeJoints([S1fst(2), S2fst(2), S3fst(2), S4fst(2)]);
        thetalist = deg2rad([S1fst(2), S2fst(2), S3fst(2), S4fst(2)]); %angles of point A
        dthetalist = deg2rad([SD1fst(2), SD2fst(2), SD3fst(2), SD4fst(2)]*0.8);
        ddthetalist = deg2rad([SDD1fst(2), SDD2fst(2), SDD3fst(2), SDD4fst(2)]);
        taulist = InverseDynamics(thetalist', dthetalist', ddthetalist', g, ...
    Ftip, Mlist, Glist, Slist)
readings = robot.getJointsReadings();

    % torque equation derived from the graph of torque v current
    torques = ((readings(3,:)*1.769 -0.2214)/1000); %removes the gravity component: td2g
        positions(i, 1:2) = taulist(2:3)';
        positions(i, 3:4) = torques(2:3)';
        i = i+1;
        pause(timestep)
        % disp(robot.getJointsReadings()); % Read joint values
        % readings = robot.getJointsReadings()
        % currents(i) = readings(3,2);
        % i = i+1;
end

%Generating the plot of joint velocity
figure
plot(positions)
title('Joint Torques Over Time')
legend('Joint 2 ID','Joint 3 ID', 'Joint 2 Real', 'Joint 3 Real')

pause(1);