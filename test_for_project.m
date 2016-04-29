clear all
close all

% copied from youbot program that illustrates the V-REP Matlab bindings.
%
% Original copyright information:
% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)

% Modifications (C) Noah Cowan 2015

disp('Program started');
%Use the following line if you had to recompile remoteApi
%vrep = remApi('remoteApi', 'extApi.h');
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);

if id < 0,
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', id);

% Make sure we close the connexion whenever the script is interrupted.
cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));

% This will only work in "continuous remote API server service"
% See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
% res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
% We're not checking the error code - if vrep is not run in continuous remote
% mode, simxStartSimulation could return an error.
% vrchk(vrep, res);

% Retrieve all handles, and stream arm and wheel joints, the robot's pose,
% the Hokuyo, and the arm tip pose.
h = ur5_init(vrep, id);
[res, h.Frame0] = vrep.simxGetObjectHandle(id, ...
'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
[res, h.Tool] = vrep.simxGetObjectHandle(id, ...
'UR5_link7', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

% Let a few cycles pass to make sure there's a value waiting for us next time
% we try to get a joint angle or the robot pose with the simx_opmode_buffer
% option.
pause(.2);

timestep = .05;

startingJoints = zeros(1,6);



res = vrep.simxPauseCommunication(id, true);
vrchk(vrep, res);

% Set the arm to its starting configuration:
for i = 1:6,
    res = vrep.simxSetJointTargetPosition(id, h.ur5Joints(i),...
        startingJoints(i),...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res, true);
end

res = vrep.simxPauseCommunication(id, false);
vrchk(vrep, res);


% Make sure everything is settled before we start
pause(2);

[res, homeGripperPosition] = ...
    vrep.simxGetObjectPosition(id, h.ur5Gripper,...
    h.ur5Ref,...
    vrep.simx_opmode_buffer);
vrchk(vrep, res, true);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% start here
p0 = [0;-0.3;0.5];
n = [0;-1;0];

% calculate the rotation matrix between the board and the base frame
z0 = [0;0;1];
axis = cross(z0,n);
angle = acos(z0.'*n/(norm(z0)*norm(n)));
axis_hat = [0,-axis(3),axis(2);axis(3),0,-axis(1);-axis(2),axis(1),0];
Rot = eye(3) + axis_hat * sin(angle) + axis_hat^2 * (1-cos(angle));

% theta0 = [90;0;0]*pi/180;
theta0 = eulerzyxinv(Rot).';
Rot = eulerxyz(theta0);
height = 0.02;% height move away from board

%move to the original of the board
T06 = [Rot,p0 + Rot*[0;0;-0.02];0 0 0 1];
Solution = ur5inv(T06);
for j = 1 : 6 
    vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
        Solution(j,4), ...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res);       
end
pause(8)
disp('Initialization Finished...')
%%%% heart
saveHeart = load('heart.mat');heart = saveHeart.heart; 
% saveWord1 = load('word1.mat');word1 = saveWord1.word1;
% saveWord2 = load('word2.mat');word2 = saveWord2.word2;
saveWord = load('word.mat');word = saveWord.word;
% patterns = {heart;word1;word2};
patterns = {heart;word};
for item = 1 : size(patterns,1)% for each pattrn
    % move to the new point with xxcm height
    T06 = [Rot,p0 + Rot*((patterns{item,1}(1,:)/100).'+[0;0;-height]);0 0 0 1];
    Solution = ur5inv(T06);
    for j = 1 : 6 % move through trajactory
        vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
            Solution(j,4),vrep.simx_opmode_oneshot);
        vrchk(vrep, res);       
    end
    pause(2)
    %
    for i = 1 : size(patterns{item,1},1)% for each point in the current pattern
        T06 = [Rot,p0 + Rot*(patterns{item,1}(i,:)/100).';0 0 0 1];
        Solution = ur5inv(T06);
        for j = 1 : 6 % move through trajactory
            vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
                Solution(j,4),vrep.simx_opmode_oneshot);
            vrchk(vrep, res);       
        end
        pause(0.1)
        if i == 1
            pause(1)
        end
        if i == size(patterns{item,1},1)% move up
            pause(2)
            T06 = [Rot,p0 + Rot*((patterns{item,1}(i,:)/100).'+[0;0;-height]);0 0 0 1];
            Solution = ur5inv(T06);
            for j = 1 : 6 % move through trajactory
                vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
                    Solution(j,4),vrep.simx_opmode_oneshot);
                vrchk(vrep, res);       
            end
            pause(0.1)
        end
    end
    pause(2)
end
theta0 = [90;0;0]*pi/180;%move back to the origin
T06 = [Rot,p0 + Rot*[0;0;-height];0 0 0 1];
Solution = ur5inv(T06);
for j = 1 : 6 %move to the original of the board
    vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
        Solution(j,4), ...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res);       
end
disp('Operation Finished')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%word1
% pause(2)
% saveWord1 = load('word1.mat');
% word1 = saveWord1.word1;
% for i = 1 : size(word1,1)
%     T06 = [eulerxyz(theta0),[0;-0.3;0.5]+eulerxyz(theta0)*(word1(i,:)/100).';0 0 0 1];
%     Solution = ur5inv(T06);
% 
%     for j = 1 : 6 %demonstrate the ith column's solution
%         vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
%             Solution(j,4), ...
%             vrep.simx_opmode_oneshot);
%         vrchk(vrep, res);       
%     end
%     pause(0.1)
% end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%word2
% pause(2)
% saveWord2 = load('word2.mat');
% word2 = saveWord2.word2;
% for i = 1 : size(word2,1)
%     T06 = [eulerxyz(theta0),[0;-0.3;0.5]+eulerxyz(theta0)*(word2(i,:)/100).';0 0 0 1];
%     Solution = ur5inv(T06);
% 
%     for j = 1 : 6 %demonstrate the ith column's solution
%         vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
%             Solution(j,4), ...
%             vrep.simx_opmode_oneshot);
%         vrchk(vrep, res);       
%     end
%     pause(0.1)
% end


%
% heart:
% [0,4.19,0;-0.12,4.37,0;-0.25,4.54,0;-0.38,4.71,0;-0.55,4.89,0;-0.73,5.05,0;-0.82,5.13,0;...
%     -0.97,5.23,0;-1.15,5.35,0;-1.30,5.44,0;-1.45,5.51,0;-1.68,5.62,0;-1.84,5.69,0;-2.02,5.76,0;...
%     -2.19,5.82,0;-2.37,5.81,0;-2.55,5.80,0;-2.72,5.78,0;-2.88,5.76,0;-3.07,5.72,0;-3.21,5.65,0;...
%     -3.32,5.61,0;-3.46,5.55,0;-3.61,5.49,0;-3.80,5.41,0;-3.98,5.31,0;-4.12,5.22,0;-4.23,5.13,0;...
%     -4.34,5.04,0;-4.48,4.93,0;-4.59,4.79,0;-4.64,4.68,0;-4.70,4.56,0;-4.76,4.39,0;-4.82,4.20,0;...
%     -4.91,3.93,0;-5.00,3.64,0;-5.08,3.43,0;-5.11,3.13,0;-5.09,2.81,0;-5.05,2.56,0;-5.03,2.37,0;...
%     -5.00,2.17,0;-4.97,1.92,0;-4.93,1.68,0;-4.85,1.46,0;-4.78,1.25,0;-4.69,0.99,0;-4.59,0.71,0;...
%     -4.48,0.50,0;-4.32,0.30,0;-4.23,0.18,0;-4.11,0.04,0;-3.89,-0.19,0;-3.63,-0.42,0;-3.42,-0.60,0;...
%     -3.22,-0.80,0;-3.05,-0.97,0;-2.88,-1.14,0;-2.68,-1.35,0;-2.37,-1.65,0;-2.21,-1.81,0;-2.07,-1.95,0;...
%     -1.90,-2.12,0;-1.74,-2.29,0;-1.58,-2.44,0;-1.41,-2.60,0;-1.22,-2.78,0;-1.07,-2.93,0;-0.91,-3.10,0;...
%     -0.73,-3.29,0;-0.64,-3.38,0;-0.51,-3.51,0;-0.35,-3.67,0;-0.18,-3.84,0;0,-4.02,0]
% word1(without_mirrored):
% [-1.15,2.37,0;-0.96,2.44,0;-0.78,2.51,0;-0.60,2.57,0;-0.43,2.64,0;-0.25,2.71,0;-0.09,2.77,0;...
%     0.12,2.85,0;0.25,2.90,0;0.42,2.96,0;0.59,3.02,0;0.77,3.09,0;0.97,3.17,0]
% word2(without_mirrored):
% [-0.09,2.77,0;-0.04,2.64,0;0.01,2.51,0;0.06,2.37,0;0.10,2.28,0;0.15,2.14,0;0.19,2.03,0;0.23,1.92,0;0.25,1.79,0;...
%     0.27,1.66,0;0.29,1.53,0;0.32,1.39,0;0.34,1.25,0;0.36,1.12,0;0.38,0.99,0;0.39,0.88,0;0.42,0.77,0;0.43,0.67,0;0.45,0.56,0;0.47,0.45,0;...
%     0.49,0.32,0;0.51,0.18,0;0.52,0.12,0;0.51,0.04,0;0.50,-0.04,0;...
%     0.48,-0.12,0;0.44,-0.19,0;0.37,-0.24,0;0.31,-0.28,0;0.22,-0.32,0;0.11,-0.35,0;0,-0.36,0;-0.12,-0.30,0;...
%     -0.20,-0.23,0;-0.28,-0.15,0;-0.34,-0.07,0;-0.38,0,0;-0.42,0.11,0;-0.46,0.22,0;-0.50,0.32,0;...
%     -0.54,0.42,0;-0.57,0.50,0;-0.60,0.59,0]
% word£º
% [-0.80,5.29,0;-0.83,5.05,0;-0.86,4.78,0;-0.95,4.51,0;-1.02,4.27,0;-1.10,4.00,0;-1.19,3.73,0;-1.29,3.40,0;-1.45,3.10,0;...
%     -1.65,2.82,0;-1.83,2.56,0;-1.99,2.35,0;-2.16,2.11,0;-2.32,1.89,0;-2.29,1.72,0;-1.99,1.77,0;-1.74,1.81,0;-1.48,1.81,0;...
%     -1.30,1.68,0;-1.27,1.48,0;-1.24,1.32,0;-1.24,1.14,0;-1.24,0.91,0;-1.24,0.71,0;-1.24,0.47,0;-1.24,0.24,0;-1.27,0.03,0;-1.29,-0.20,0;...
%     -1.38,-0.47,0;-1.42,-0.87,0;-1.34,-0.47,0;-1.22,-0.13,0;-1.08,0.09,0;-0.90,0.34,0;-0.71,0.54,0;-0.49,0.73,0;-0.30,0.89,0;-0.11,1.05,0;...
%     0.09,1.23,0;0.31,1.41,0;0.57,1.58,0;0.79,1.72,0;0.97,1.84,0;1.13,1.94,0;1.28,2.04,0;1.43,2.17,0;1.57,2.28,0;1.65,2.38,0;...
%     1.65,2.51,0;1.54,2.62,0;1.38,2.72,0;1.19,2.78,0;1.01,2.84,0;0.79,2.75,0;0.79,2.62,0;0.75,2.38,0;0.70,2.11,0;0.66,1.86,0;...
%     0.62,1.68,0;0.61,1.51,0;0.61,1.34,0;0.61,1.11,0;0.61,0.91,0;0.61,0.73,0;0.59,0.54,0;0.57,0.35,0;0.52,0.13,0;0.40,0.09,0;...
%     0.31,0.19,0;0.28,0.43,0;0.35,0.62,0;0.54,0.77,0;0.70,0.71,0;0.70,0.62,0;0.60,0.49,0;0.46,0.34,0;0.34,0.20,0;0.21,0.10,0;...
%     0.09,-0.01,0;0.09,-0.05,0;0.31,-0.01,0;0.51,0.09,0;0.70,0.18,0;0.85,0.18,0;0.82,0.04,0;0.65,-0.09,0;0.50,-0.20,0;0.31,-0.35,0;...
%     0.15,-0.48,0;0,-0.59,0;-0.07,-0.65,0;-0.07,-0.74,0;0.04,-0.76,0;0.21,-0.69,0;0.39,-0.62,0;0.62,-0.57,0;0.84,-0.51,0;1.22,-0.40,0]
