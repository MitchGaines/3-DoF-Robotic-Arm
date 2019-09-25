function plotArm(q, ef_vel)
    T01 = tdh(-q(1), 0.135, 0, -90);
    T12 = tdh(-q(2), 0, 0.175, 0);
    T23 = tdh(-q(3)+90, 0, 0.16928, 90);
    
    fwkin2 = [T01(1,4) T01(2,4) T01(3,4)];
    T012 = T01 * T12;
    fwkin3 = [T012(1,4) T012(2,4) T012(3,4)];
    fwkinEF = fwkin3001(q);
    
    plot3([0, fwkin2(1), fwkin3(1), fwkinEF(1), ef_vel(1)], [0, fwkin2(2), fwkin3(2), fwkinEF(2), ef_vel(2)], [0, fwkin2(3), fwkin3(3), fwkinEF(3), ef_vel(3)]);
    xlim([-0.5 0.5])
    ylim([-0.5 0.5])
    zlim([-0.5 0.5])
    
end