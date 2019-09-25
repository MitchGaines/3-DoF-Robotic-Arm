function J = jacob0(q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
 
 
 theta_1 = q(1)*pi/180;
 theta_2 = q(2)*pi/180;
 theta_3 = q(3)*pi/180;
 L1 = 0.135; L2 = 0.175; L3 = 0.16928;
 JV = [L3*sin(theta_1)*sin(theta_2)*sin(theta_3) - L3*cos(theta_2)*cos(theta_3)*sin(theta_1) - L2*cos(theta_2)*sin(theta_1), - L2*cos(theta_1)*sin(theta_2) - L3*cos(theta_1)*cos(theta_2)*sin(theta_3) - L3*cos(theta_1)*cos(theta_3)*sin(theta_2), - L3*cos(theta_1)*cos(theta_2)*sin(theta_3) - L3*cos(theta_1)*cos(theta_3)*sin(theta_2); L3*cos(theta_1)*sin(theta_2)*sin(theta_3) - L3*cos(theta_1)*cos(theta_2)*cos(theta_3) - L2*cos(theta_1)*cos(theta_2), L2*sin(theta_1)*sin(theta_2) + L3*cos(theta_2)*sin(theta_1)*sin(theta_3) + L3*cos(theta_3)*sin(theta_1)*sin(theta_2), L3*cos(theta_2)*sin(theta_1)*sin(theta_3) + L3*cos(theta_3)*sin(theta_1)*sin(theta_2); sym(0), L2*cos(theta_2) - L3*cos(theta_2)*cos(theta_3) - L3*sin(theta_2)*sin(theta_3), L3*cos(theta_2)*cos(theta_3) + L3*sin(theta_2)*sin(theta_3)];
 JV = double(vpa(JV,4));
  
  T0_1 = tdh(-q(1), 0.135, 0, -90);
  T0_2 = tdh(-q(1), 0.135, 0, -90) * tdh(-q(2), 0, 0.175, 0);
  
  % z0
  JW(1,1) = 0;
  JW(2,1) = 0;
  JW(3,1) = 1;
  % z1
  JW(1,2) = T0_1(1,3); 
  JW(2,2) = T0_1(2,3);
  JW(3,2) = T0_1(3,3);
  %z2
  JW(1,3) = T0_2(1,3); 
  JW(2,3) = T0_2(2,3);
  JW(3,3) = T0_2(3,3);
  
  J = [JV;JW];
  Jvd = det(JV) 
 

end

