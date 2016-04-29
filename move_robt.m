function move_robt(id,vrep,h,theta,Socket_conn,REAL)
% Andrew Spielvogel, Erin Sutton Nov 2015
%
% function for moving robot in sim or hardware
% change 


if REAL
% Here, gst0G is the initial base frame, using the D-H convention
d1=0.0892;a2=0.425;a3=0.392;d4=0.1093;d5=0.09475;d6=0.0825; 

% parameters
R0 = eye(3); 
p0 = [0;0;0];
gst0 = [R0 p0; 0 0 0 1];

RG = eulerzyx([pi/2;0;0]);
pG = [-a2-a3;-d4-d6;d1-d5];
gst0G = [RG pG; 0 0 0 1] * gst0;

% Somehow, we have an error in how we attached our tool frame, and
% this corrects it:
gNoah = [ ...
    -1   0   0   0;
    0    1   0   0;
    0    0  -1   0;
    0    0   0   1] * ...
    [eye(3,3) [0 0 0.165]';
        0 0 0 1];

% set all the twist axes and point in each axis
w(:,1) = [0;0;1];
w(:,2) = [0;-1;0];
w(:,3) = [0;-1;0];
w(:,4) = [0;-1;0];
w(:,5) = [0;0;1];
w(:,6) = [0;-1;0];

q(:,1) = [0;0;0];
q(:,2) = [0;0;d1];
q(:,3) = [-a2;0;d1];
q(:,4) = [-a2-a3;0;d1];
q(:,5) = [-a2-a3;-d4;d1];
q(:,6) = [-a2-a3;-d4;d1-d5];

% do forward kinematics
gd = ur5fwd( gst0G, w, q, 6, theta' )*gNoah;    

Trans_d = gd(1:3,4);
w_d = vrrotmat2vec(gd(1:3,1:3));
Robot_Pose = readrobotpose(Socket_conn);
vrobot = Robot_Pose(1:3); % in m
wrobot = Robot_Pose(4:6);
Orien_d = (w_d(4)*w_d(1:3))';

    moverobot(Socket_conn,Trans_d,Orien_d);
else
    set_joints(id,vrep,theta,h);
end