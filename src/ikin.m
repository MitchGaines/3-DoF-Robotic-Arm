function joint_angles = ikin (end_pos)
    L1 = 135;
    L2 = 175;
    L3 = 169.28;
    
    joint_angles = [];
    joint_angles(1) = atan2d(end_pos(2), end_pos(1));
    
    s = end_pos(3) - L1;
    r = sqrt(end_pos(1)^2 + end_pos(2)^2);
    h = sqrt(r^2 + s^2);
    
    joint_angles(2) = acosd((h^2 + L2^2 - L3^2)/(2*h*L2)) + atand(s/r);
    
    joint_angles(3) = acosd((L2^2 + L3^2 - h^2)/(2*L2*L3)) - 90;
    
    max_base = ticksToAngle(975);
    min_base = ticksToAngle(-1010);
    max_elbow = ticksToAngle(1170);
    min_elbow = ticksToAngle(-54);
    max_wrist = ticksToAngle(2428);
    min_wrist = ticksToAngle(-274);
    
    if(joint_angles(1) > max_base || joint_angles(1) < min_base)
        error("Base joint angle out of bounds")
    elseif(joint_angles(2) > max_elbow || joint_angles(2) < min_elbow)
        error("Elbow joint angle out of bounds")
    elseif(joint_angles(3) > max_wrist || joint_angles(3) < min_wrist)
        error("Wrist joint angle out of bounds")
    end

end

