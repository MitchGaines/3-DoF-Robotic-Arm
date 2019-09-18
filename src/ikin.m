function joint_angles = ikin (end_pos)
    L1 = 135;
    L2 = 175;
    L3 = 169.28;
    
    joint_angles = [];
    joint_angles(1) = atan2(end_pos(2), end_pos(1));
    
    s = end_pos(3) - L1;
    r = sqrt(end_pos(1)^2 + end_pos(2)^2);
    h = sqrt(r^2 + s^2);
    
    joint_angles(2) = acos((h^2 + L2^2 - L3^2)/(2*h*L2)) + atan2(s/r);
    
    joint_angles(3) = acos((L2^2 + L3^2 - h^2)/(2*L2*L3)) - 90;
    
end

