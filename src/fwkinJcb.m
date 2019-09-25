function x = fwkinJcb(joint_ang, joint_vel)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

x = jacob0(joint_ang) * joint_vel;
end

