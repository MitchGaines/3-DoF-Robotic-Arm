function J = jacob0(q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
 
 
 theta_1 = q(1)*pi/180;
 theta_2 = q(2)*pi/180;
 theta_3 = q(3)*pi/180;
 L1 = 0.135; L2 = 0.175; L3 = 0.16928;

 JV = [ (28251218388874341*cos(theta_1)*sin(theta_2))/360287970189639680 - (7*cos(theta_2)*sin(theta_1))/40 + (529*cos(theta_3 - 90)*((4035888341267763*cos(theta_1)*sin(theta_2))/9007199254740992 - cos(theta_2)*sin(theta_1)))/3125 + (529*sin(theta_3 - 90)*((4035888341267763*cos(theta_1)*cos(theta_2))/9007199254740992 + sin(theta_1)*sin(theta_2)))/3125, (28251218388874341*cos(theta_2)*sin(theta_1))/360287970189639680 - (7*cos(theta_1)*sin(theta_2))/40 - (529*cos(theta_3 - 90)*(cos(theta_1)*sin(theta_2) - (4035888341267763*cos(theta_2)*sin(theta_1))/9007199254740992))/3125 - (529*sin(theta_3 - 90)*(cos(theta_1)*cos(theta_2) + (4035888341267763*sin(theta_1)*sin(theta_2))/9007199254740992))/3125, - (529*cos(theta_3 - 90)*(cos(theta_1)*sin(theta_2) - (4035888341267763*cos(theta_2)*sin(theta_1))/9007199254740992))/3125 - (529*sin(theta_3 - 90)*(cos(theta_1)*cos(theta_2) + (4035888341267763*sin(theta_1)*sin(theta_2))/9007199254740992))/3125; ...
        (529*sin(theta_3 - 90)*(cos(theta_1)*sin(theta_2) - (4035888341267763*cos(theta_2)*sin(theta_1))/9007199254740992))/3125 - (28251218388874341*sin(theta_1)*sin(theta_2))/360287970189639680 - (529*cos(theta_3 - 90)*(cos(theta_1)*cos(theta_2) + (4035888341267763*sin(theta_1)*sin(theta_2))/9007199254740992))/3125 - (7*cos(theta_1)*cos(theta_2))/40, (28251218388874341*cos(theta_1)*cos(theta_2))/360287970189639680 + (7*sin(theta_1)*sin(theta_2))/40 + (529*cos(theta_3 - 90)*((4035888341267763*cos(theta_1)*cos(theta_2))/9007199254740992 + sin(theta_1)*sin(theta_2)))/3125 - (529*sin(theta_3 - 90)*((4035888341267763*cos(theta_1)*sin(theta_2))/9007199254740992 - cos(theta_2)*sin(theta_1)))/3125,   (529*cos(theta_3 - 90)*((4035888341267763*cos(theta_1)*cos(theta_2))/9007199254740992 + sin(theta_1)*sin(theta_2)))/3125 - (529*sin(theta_3 - 90)*((4035888341267763*cos(theta_1)*sin(theta_2))/9007199254740992 - cos(theta_2)*sin(theta_1)))/3125; ... 
        0, (28183421287433573*cos(theta_2))/180143985094819840 + (2129861408721765731*cos(theta_3 - 90)*cos(theta_2))/14073748835532800000 - (2129861408721765731*sin(theta_3 - 90)*sin(theta_2))/14073748835532800000, (2129861408721765731*cos(theta_3 - 90)*cos(theta_2))/14073748835532800000 - (2129861408721765731*sin(theta_3 - 90)*sin(theta_2))/14073748835532800000];
 
 JV = double(vpa(JV, 4));
 
 T0_1 = tdhr(-q(1), 0.135, 0, -90);
 T0_2 = tdhr(-q(1), 0.135, 0, -90) * tdhr(-q(2), 0, 0.175, 0);
  
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

end

