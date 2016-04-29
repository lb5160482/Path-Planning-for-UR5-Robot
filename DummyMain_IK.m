% Dummy Main Function
clear all
close all
% Must have these two lines:
Socket_conn = 1;  
REAL = 0;               % set to 1 to test on real robot

if ~REAL
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
h = ur5_init(vrep, id);
[res, h.Frame0] = vrep.simxGetObjectHandle(id, ...
'Frame0', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
[res, h.Tool] = vrep.simxGetObjectHandle(id, ...
'UR5_link7', vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);
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

pause(2)    
else
    Robot_IP = '172.22.22.2';
    Socket_conn = tcpip(Robot_IP,30000,'NetworkRole','server');
    fclose(Socket_conn);
    disp('Press play on robot')
    fopen(Socket_conn);
    disp('Connected');
    id = 1;
    vrep = 0;
    h=0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p0 = [0;-0.3;0.5];
n = [0;1;1];

% calculate the rotation matrix between the board and the base frame
z0 = [0;0;1];% body frame normal
axis = cross(z0,-n);% rotation axis
angle = acos(z0.'*-n/(norm(z0)*norm(-n))); % roatation angle
axis_hat = [0,-axis(3),axis(2);axis(3),0,-axis(1);-axis(2),axis(1),0];
Rot = eye(3) + axis_hat * sin(angle) + axis_hat^2 * (1-cos(angle));

theta0 = eulerzyxinv(Rot).';% Euler Angle
Rot = eulerxyz(theta0); % rotation matrix between the body frame and space frame
height = 0.02;% height move away from board

% move to the above of the original of the board with exactly 'height''s height
T06 = [Rot,p0 + Rot*[0;0;-0.02];0 0 0 1];
Solution = ur5inv(T06); % use IK to get the joint angles
theta = Solution(:,4).';
move_robt(id,vrep,h,theta,Socket_conn,REAL);
pause(3)
disp('Initialization Finished...')
%%%% read the pattern we want to draw
saveHeart = load('heart.mat');heart = saveHeart.heart; 
saveWord = load('word.mat');word = saveWord.word;
patterns = {heart;word};
for item = 1 : size(patterns,1)% for each pattrn
    % move to the new point with xxcm height
    T06 = [Rot,p0 + Rot*((patterns{item,1}(1,:)/100).'+[0;0;-height]);0 0 0 1];
    Solution = ur5inv(T06);
    theta = Solution(:,4).';
    move_robt(id,vrep,h,theta,Socket_conn,REAL);
    pause(2)
    %
    for i = 1 : size(patterns{item,1},1)% for each point in the current pattern
        T06 = [Rot,p0 + Rot*(patterns{item,1}(i,:)/100).';0 0 0 1];
        Solution = ur5inv(T06);
        theta = Solution(:,4).';
        move_robt(id,vrep,h,theta,Socket_conn,REAL);
        pause(0.1)
        if i == 1
            pause(1)
        end
        if i == size(patterns{item,1},1)% move up
            pause(2)
            T06 = [Rot,p0 + Rot*((patterns{item,1}(i,:)/100).'+[0;0;-height]);0 0 0 1];
            Solution = ur5inv(T06);
            theta = Solution(:,4).';
            move_robt(id,vrep,h,theta,Socket_conn,REAL);
            pause(0.1)
        end
    end
    pause(2)
end
theta0 = [90;0;0]*pi/180;%move back to the origin
T06 = [Rot,p0 + Rot*[0;0;-height];0 0 0 1];
Solution = ur5inv(T06);
theta = Solution(:,4).';
move_robt(id,vrep,h,theta,Socket_conn,REAL);
disp('Operation Finished')





