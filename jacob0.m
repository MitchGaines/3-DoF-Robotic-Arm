function J = jacob0(q)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

 DH = tdh(-q(1), 0.135, 0, -90) * tdh(-q(2), 0, 0.175, 0) * tdh(-q(3)+90, 0, 0.16928, 90);
 
 x_vect = [DH(1,4), DH(2,4), DH(3,4)];
 
 [J1] = gradient(x_vect, q(1));
 [J2] = gradient(x_vect, q(2));
 [J3] = gradient(x_vect, q(3));
 
 J = zeros(6, 3);
 J(1,1) = J1(1);
 J(2,1) = J1(2);
 J(3,1) = J1(3);
 
 J(1,2) = J2(1);
 J(2,2) = J2(2);
 J(3,2) = J2(3);
 
 J(1,3) = J3(1);
 J(2,3) = J3(2);
 J(3,3) = J3(3);
 
%  J = zeros(6, 3);
%  J(1,1) = gradient(x_vect(1), x);
%  J(2,1) = gradient(x_vect(2), x);
%  J(3,1) = gradient(x_vect(3), x);
%  
%  J(1,2) = gradient(x_vect(1), y);
%  J(2,2) = gradient(x_vect(2), y);
%  J(3,2) = gradient(x_vect(3), y);
%  
%  J(1,3) = gradient(x_vect(1), z);
%  J(2,3) = gradient(x_vect(2), z);
%  J(3,3) = gradient(x_vect(3), z);
 
  x = q(1);
  y = q(2);
  z = q(3);

 
 
  T0_1 = tdh(-q(1), 0.135, 0, -90);
  T0_2 = tdh(-q(1), 0.135, 0, -90) * tdh(-q(2), 0, 0.175, 0);
  
  % z0
  J(4,1) = 0;
  J(5,1) = 0;
  J(6,1) = 1;
  % z1
  J(4,2) = T0_1(1,3); 
  J(5,2) = T0_1(2,3);
  J(6,2) = T0_1(3,3);
  %z2
  J(4,3) = T0_2(1,3); 
  J(5,3) = T0_2(2,3);
  J(6,3) = T0_2(3,3);
  

 

end

