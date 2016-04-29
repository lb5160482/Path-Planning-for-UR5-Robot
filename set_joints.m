function set_joints(id,vrep,theta,h)
for j = 1 : 6 
    vrep.simxSetJointTargetPosition(id, h.ur5Joints(j),...
        theta(j),vrep.simx_opmode_oneshot);
%     vrchk(vrep, res);       
end