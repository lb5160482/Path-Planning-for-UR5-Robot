function g = get_pose(id,vrep,h,Socket_conn,REAL)
% Andrew Spielvogel Nov 2015
%
% get pose



if REAL
    Robot_Pose = readrobotpose(Socket_conn);
    v = Robot_Pose(1:3); % in m
    w = Robot_Pose(4:6);
    
    %compute rotation matrix
    wskew = skew(w);
    R = expm(wskew);
    
    %build homogenous transformation, g
    g = [R v';
    0 0 0 1];
else
    g = getf(id,vrep,h.ur5Joints(end));
end