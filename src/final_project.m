clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

cam = webcam();
LoadCalibrationImages();
pause(.5);
T_cam_to_checker = getCamToCheckerboard(cam, cameraParams);
disp('REMOVE CHECKERBOARD');
pause(.5);
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
try
  MOVE_SERV_ID = 01;
  PID_CONFIG_SERV_ID = 02;
  STATUS_SERV_ID = 03;
  GRIPPER_SERVER_ID = 05;

  DEBUG   = false;          % enables/disables debug prints

  tic
  actualPosition = 0;
  holding_ball = 0;
  pid_config_packet(1) = 0.00295; % base kp
  pid_config_packet(2) = 0.00010; % base ki
  pid_config_packet(3) = 0.00002; % base kd
  pid_config_packet(4) = 0.007; % elbow kp
  pid_config_packet(5) = 0.00007; % elbow ki
  pid_config_packet(6) = 0.00008; % elbow kd
  pid_config_packet(7) = 0.00400; % wrist kp
  pid_config_packet(8) = 0.00007; % wrist ki
  pid_config_packet(9) = 0.00002; % wrist kd
  pp.write(PID_CONFIG_SERV_ID, pid_config_packet);
  pp.write(GRIPPER_SERVER_ID, [2 0 0]);
  
  joint_tolerance = 5;
  position_tolerance = 3;
%   step = 0.01;
%   total_move_step = 19;
  step = 0.004;
  total_move_step = 10;
  
  rapid_packet = zeros(15, 1, 'single');
  left_packet = zeros(15, 1, 'single');
  right_packet = zeros(15, 1, 'single');
  
  rapid_pos_goal = [200 0 100];
  left_pos_goal = [0 210 0];
  right_pos_goal = [0 -210 0];
  
  rapid_joint_goal = ikin(rapid_pos_goal);
  left_joint_goal = ikin(left_pos_goal); 
  right_joint_goal = ikin(right_pos_goal);

  previous_ball_locations = zeros(3, 2, 'single');
  
  armcontrol = 'Rapid';
  color_hold = '';
  color_it = 'Grn';
  
  packet = zeros(15, 1, 'single');
  pp.write(STATUS_SERV_ID, packet);
  pause(0.003);
  returnPacket = pp.read(STATUS_SERV_ID); % gets current position data
  jointAngles = [ticksToAngle(returnPacket(1)) ticksToAngle(returnPacket(2)) ticksToAngle(returnPacket(3))];
  
  base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), rapid_joint_goal(1));
  elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), rapid_joint_goal(2));
  wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), rapid_joint_goal(3));
  
  green_loc = [0, 0];
  yellow_loc = [0, 0];
  blue_loc = [0, 0];
  
  while true
      current_image = snapshot(cam);
      packet = zeros(15, 1, 'single');
      pp.write(STATUS_SERV_ID, packet);
      pause(0.003);
      returnPacket = pp.read(STATUS_SERV_ID); % gets current position data
      jointAngles = [ticksToAngle(returnPacket(1)) ticksToAngle(returnPacket(2)) ticksToAngle(returnPacket(3))];
      current_position = 1000 * fwkin3001(jointAngles);
      
      tmp = [green_loc; blue_loc; yellow_loc];
      [green_loc, blue_loc, yellow_loc] = findObjs(current_image, T_cam_to_checker, cameraParams);
      % BALL LOCATIONS ALWAYS USES GREEN BLUE YELLOW AS ITS ORDER
      
      if(~outsideOfTolerances(green_loc, [-478, 528], 40) && strcmp(armcontrol, 'Grn'))
        % my green location within the tolerances of the arbitrary not
        % shown location? If so, just keep it as my last known green
        % coordinate
        green_loc = tmp(1,:);
      end 
      if(~outsideOfTolerances(blue_loc, [-478, 528], 40) && strcmp(armcontrol, 'Blu'))
        % my blue location within the tolerances of the arbitrary not
        % shown location? If so, just keep it as my last known blue
        % coordinate
        blue_loc = tmp(2,:);
      end 
      if(~outsideOfTolerances(yellow_loc, [-478, 528], 40) && strcmp(armcontrol, 'Ylw'))
        % my yellow location within the tolerances of the arbitrary not
        % shown location? If so, just keep it as my last known yellow
        % coordinate
        yellow_loc = tmp(3,:);
      end 
      
      ball_locations = [green_loc; blue_loc; yellow_loc];
      ball_locations = getCheckerboardToRobot(ball_locations);
      
      green_goal =  ikin([ball_locations(1, 1) ball_locations(1, 2) -20]);
      blue_goal = ikin([ball_locations(2, 1) ball_locations(2, 2) -20]);
      yellow_goal = ikin([ball_locations(3, 1) ball_locations(3, 2) -20]);
      
      switch armcontrol
          
          case 'Grn'
            if outsideOfTolerances(previous_ball_locations(1,:), ball_locations(1,:), 5)
                disp('Recalculating coefficients. . . ');
                green_goal =  ikin([ball_locations(1, 1) ball_locations(1, 2) -20]);
            
                base_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(1), green_goal(1));
                elbow_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(2), green_goal(2));
                wrist_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(3), green_goal(3));
            
            else
                move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
                move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
                move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

                pp.write(MOVE_SERV_ID, move_packet);
            end
            
            if atLocation(jointAngles, green_goal) 
                armcontrol = 'Grip';
                color_hold = 'Grn';
                color_it = 'Blu';
            end 
            
          case 'Blu'
            if outsideOfTolerances(previous_ball_locations(2,:), ball_locations(2,:), 5)
                disp('Recalculating coefficients. . . ');
                blue_goal = ikin([ball_locations(2, 1) ball_locations(2, 2) -20]);
            
                base_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(1), blue_goal(1));
                elbow_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(2), blue_goal(2));
                wrist_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(3), blue_goal(3));
            
            else
                move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
                move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
                move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

                pp.write(MOVE_SERV_ID, move_packet);
            end
            
            if atLocation(jointAngles, blue_goal) 
                armcontrol = 'Grip';
                color_hold = 'Blu';
                color_it = 'Ylw';
            end 

          case 'Ylw'
            if outsideOfTolerances(previous_ball_locations(3,:), ball_locations(3,:), 5)
                disp('Recalculating coefficients. . . ');
                yellow_goal = ikin([ball_locations(3, 1) ball_locations(3, 2) -20]);
            
                base_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(1), yellow_goal(1));
                elbow_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(2), yellow_goal(2));
                wrist_coef = trajectory(toc, toc+total_move_step-3, 0, 0, jointAngles(3), yellow_goal(3));
            
            else
                move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
                move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
                move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

                pp.write(MOVE_SERV_ID, move_packet);
            end
            
            if atLocation(jointAngles, yellow_goal) 
                armcontrol = 'Grip';
                color_hold = 'Yellow';
                color_it = 'Grn';
            end 
            
          case 'Grip'
              holding_ball = 1;
              armcontrol = 'Rapid';
              grip_step = 1;
              
              pp.write(GRIPPER_SERVER_ID, [1 0 0]);
              pause(1);

              base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), rapid_joint_goal(1));
              elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), rapid_joint_goal(2));
              wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), rapid_joint_goal(3));
              
          case 'Rapid'
              move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
              move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
              move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

              pp.write(MOVE_SERV_ID, move_packet);
              
              if atLocation(jointAngles, rapid_joint_goal)
                  if holding_ball
                    armcontrol = 'Sort';
                    base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), left_joint_goal(1));
                    elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), left_joint_goal(2));
                    wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), left_joint_goal(3));
                  else
                    armcontrol = color_it;
                    switch color_it
                        case 'Grn'
                            base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), green_goal(1));
                            elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), green_goal(2));
                            wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), green_goal(3));
                        case 'Blu'
                            base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), blue_goal(1));
                            elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), blue_goal(2));
                            wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), blue_goal(3));
                        case 'Ylw'
                            base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), yellow_goal(1));
                            elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), yellow_goal(2));
                            wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), yellow_goal(3));
                    end
                  end
              end
                
          case 'Sort' 
            move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

            pp.write(MOVE_SERV_ID, move_packet);
            
            if atLocation(jointAngles, left_joint_goal) || atLocation(jointAngles, right_joint_goal)
                holding_ball = 0;
                pp.write(GRIPPER_SERVER_ID, [2 0 0]);
                armcontrol = 'Rapid';
                color_hold = '';
                
                base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), rapid_joint_goal(1));
                elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), rapid_joint_goal(2));
                wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), rapid_joint_goal(3));
            end  
      end
      
      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      previous_ball_locations = ball_locations;
  end 
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
