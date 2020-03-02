function location = atLocation(jointAngles, jointGoal)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
joint_tolerance = 4;
if (jointAngles(1) < jointGoal(1)+joint_tolerance && jointAngles(1) > jointGoal(1)-joint_tolerance) &&  (jointAngles(2) < jointGoal(2)+joint_tolerance && jointAngles(2) > jointGoal(2)-joint_tolerance) && (jointAngles(3) < jointGoal(3)+joint_tolerance && jointAngles(3) > jointGoal(3)-joint_tolerance)
    location = true;
else
    location = false;
end

end

