function handles = ur5_init(vrep, id)
% Initialize ur5 -- based on code written for youbot

% Original (C) info here:
% (C) Copyright Renaud Detry 2013.
% Distributed under the GNU General Public License.
% (See http://www.gnu.org/copyleft/gpl.html)
% 
% All modifications (C) Copyright Noah Cowan 2015

% Retrieve all handles, and streams joints and pose of UR5

handles = struct('id', id);

jointNames={'UR5_joint1','UR5_joint2','UR5_joint3','UR5_joint4',...
    'UR5_joint5','UR5_joint6'};
% efName = 'UR5_link7';
% [res, link7] = vrep.simxGetObjectHandle(id, ...
%     efName, vrep.simx_opmode_oneshot_wait);
% vrchk(vrep, res);
% handles.link7 = link7;

ur5Joints = -ones(1,6); 
for i = 1:6
    [res, ur5Joints(i)] = vrep.simxGetObjectHandle(id, ...
        jointNames{i}, vrep.simx_opmode_oneshot_wait);
    vrchk(vrep, res);
end
for i = 1:6,
    vrep.simxSetJointTargetVelocity(id, ur5Joints(i),0, ...
        vrep.simx_opmode_oneshot);
    vrchk(vrep, res);
end

handles.ur5Joints = ur5Joints;


[res, ur5Ref] = vrep.simxGetObjectHandle(id, 'UR5', ...
    vrep.simx_opmode_oneshot_wait); 
vrchk(vrep, res);

[res, ur5Gripper] = vrep.simxGetObjectHandle(id, 'UR5_connection', ...
    vrep.simx_opmode_oneshot_wait);
vrchk(vrep, res);

handles.ur5Ref = ur5Ref;
handles.ur5Gripper = ur5Gripper;



% Stream wheel angles, Hokuyo data, and robot pose (see usage below)
% Wheel angles are not used in this example, but they may/will be necessary in
% your project.
for i = 1:6,
  res = vrep.simxGetJointPosition(id, ur5Joints(i),...
      vrep.simx_opmode_streaming); 
  vrchk(vrep, res, true);
end
res = vrep.simxGetObjectPosition(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Ref, -1,...
    vrep.simx_opmode_streaming); 
vrchk(vrep, res, true);

% Stream the arm joint angles and the tip position/orientation
res = vrep.simxGetObjectPosition(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
res = vrep.simxGetObjectOrientation(id, ur5Gripper, ur5Ref, vrep.simx_opmode_streaming); vrchk(vrep, res, true);
for i = 1:6,
  res = vrep.simxGetJointPosition(id, ur5Joints(i),...
      vrep.simx_opmode_streaming);
  vrchk(vrep, res, true);
end

vrep.simxGetPingTime(id); % make sure that all streaming data has reached the client at least once

end
