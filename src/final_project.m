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
  left_pos_goal = [0 250 0];
  right_pos_goal = [0 -250 0];
  
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
  arc_goal = [0, 0];
  
  %Disk size calculation variables
  diskMin = 50;
  diskMax = 150;
  diskCenter = [0, 0];
  diskRadius = 0;
  diskSize = 0; % 0 -> small disk, 1 -> large disk
  
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
      
      green_goal =  ikin([ball_locations(1, 1) ball_locations(1, 2) 10]);
      blue_goal = ikin([ball_locations(2, 1) ball_locations(2, 2) 10]);
      yellow_goal = ikin([ball_locations(3, 1) ball_locations(3, 2) 10]);
      
      switch armcontrol
          case 'Grn'
            disp("Pursuing Green");
            if outsideOfTolerances(previous_ball_locations(1,:), ball_locations(1,:), 5)
                disp('Recalculating coefficients. . . ');
                green_goal =  ikin([ball_locations(1, 1) ball_locations(1, 2) 10]);
            
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
                armcontrol = 'Descend';
                color_hold = 'Grn';
                color_it = 'Blu';
            end 
            
          case 'Blu'
            disp("Pursuing Blue");
            if outsideOfTolerances(previous_ball_locations(2,:), ball_locations(2,:), 5)
                disp('Recalculating coefficients. . . ');
                blue_goal = ikin([ball_locations(2, 1) ball_locations(2, 2) 10]);
            
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
                armcontrol = 'Descend';
                color_hold = 'Blu';
                color_it = 'Ylw';
            end 

          case 'Ylw'
            disp("Pursuing Yellow");
            if outsideOfTolerances(previous_ball_locations(3,:), ball_locations(3,:), 5)
                disp('Recalculating coefficients. . . ');
                yellow_goal = ikin([ball_locations(3, 1) ball_locations(3, 2) 10]);
            
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
                armcontrol = 'Descend';
                color_hold = 'Ylw';
                color_it = 'Grn';
            end 
            
          case 'Descend'
              disp("Descending");
              
              arc_goal = ikin([current_position(1) -current_position(2) current_position(3)-35]);

              base_coef = trajectory(toc, toc+3, 0, 0, jointAngles(1), arc_goal(1));
              elbow_coef = trajectory(toc, toc+3, 0, 0, jointAngles(2), arc_goal(2));
              wrist_coef = trajectory(toc, toc+3, 0, 0, jointAngles(3), arc_goal(3));
              
              armcontrol = 'Grip';
              
          case 'Grip'
              disp("Pursuing Grip");
              move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
              move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
              move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

              pp.write(MOVE_SERV_ID, move_packet);
              
              if atLocation(jointAngles, arc_goal)
                  holding_ball = 1;
                  armcontrol = 'Rapid';
                  grip_step = 1;

                  pp.write(GRIPPER_SERVER_ID, [1 0 0]);
                  pause(1);

                  base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), rapid_joint_goal(1));
                  elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), rapid_joint_goal(2));
                  wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), rapid_joint_goal(3));
              end 
          case 'Rapid'
              disp("Pursuing Rapid");
              move_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
              move_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
              move_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));

              pp.write(MOVE_SERV_ID, move_packet);
              
              if atLocation(jointAngles, rapid_joint_goal)
                  if holding_ball
                    armcontrol = 'Classify';
                    
                  else
                      
                    if strcmp(color_it, 'Grn') && ~outsideOfTolerances(green_loc, [-478, 528], 40)
                        color_it = 'Blu';
                        armcontrol = color_it;
                    elseif strcmp(color_it, 'Blu') && ~outsideOfTolerances(blue_loc, [-478, 528], 40)
                        color_it = 'Ylw';
                        armcontrol = color_it;
                    elseif strcmp(color_it, 'Ylw') && ~outsideOfTolerances(yellow_loc, [-478, 528], 40)
                        color_it = 'Grn';
                        armcontrol = color_it;
                    else 
                        armcontrol = color_it;
                    end
                    
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
                  delay = 0;
              end
              
          case 'Classify'
              if delay == 0
                  delay = 1;
              elseif delay == 1                  
                  disp ("Pursuing Classify");
                  cropped_image = imcrop(current_image, 'logical', [186 101 234 200]);
                  [diskCenter, diskRadius] = imfindcircles(cropped_image,[diskMin diskMax],'ObjectPolarity', 'dark', 'Sensitivity',0.95, 'EdgeThreshold', .1);
                  disp("Center");
                  disp(diskCenter);
                  disp("Radius");              
                  disp(diskRadius); 
                  if(diskRadius >= 75)
                      diskSize = 1;
                      armcontrol = 'SortBig';
                      base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), left_joint_goal(1));
                      elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), left_joint_goal(2));
                      wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), left_joint_goal(3));
                  else
                      diskSize = 0;
                      armcontrol = 'SortSmall';
                      base_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(1), right_joint_goal(1));
                      elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(2), right_joint_goal(2));
                      wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, jointAngles(3), right_joint_goal(3));
                  end
              end
         
          case 'SortBig' 
            disp("Pursuing SortBig");
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
         
          case 'SortSmall' 
            disp("Pursuing SortSmall");
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
