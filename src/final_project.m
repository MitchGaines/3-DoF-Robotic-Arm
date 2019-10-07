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
pause(1);
T_cam_to_checker = getCamToCheckerboard(cam, cameraParams);
disp('REMOVE CHECKERBOARD');
pause(3);
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
try
  MOVE_SERV_ID = 01;
  PID_CONFIG_SERV_ID = 02;
  STATUS_SERV_ID = 03; 

  DEBUG   = false;          % enables/disables debug prints

  tic
  actualPosition = 0;
  pid_config_packet(1) = 0.00295; % base kp
  pid_config_packet(2) = 0.00010; % base ki
  pid_config_packet(3) = 0.00002; % base kd
  pid_config_packet(4) = 0.01; % elbow kp
  pid_config_packet(5) = 0.00004; % elbow ki
  pid_config_packet(6) = 0.00005; % elbow kd
  pid_config_packet(7) = 0.00400; % wrist kp
  pid_config_packet(8) = 0.00007; % wrist ki
  pid_config_packet(9) = 0.00002; % wrist kd
  pp.write(PID_CONFIG_SERV_ID, pid_config_packet);
  
  joint_tolerance = 5;
  position_tolerance = 3;
  step = 0.001;
  total_move_step = 7;
  
  rapid_packet = zeros(15, 1, 'single');
  left_packet = zeros(15, 1, 'single');
  right_packet = zeros(15, 1, 'single');
  
  rapid_pos_goal = [150 0 300];
  left_pos_goal = [150 -200 0];
  right_pos_goal = [150 200 0];
  
  rapid_joint_goal = ikin(rapid_pos_goal);
  left_joint_goal = ikin(left_pos_goal); 
  right_joint_goal = ikin(right_pos_goal);
  
  rapid_packet(1) = angleToTicks(rapid_joint_goal(1));
  rapid_packet(4) = angleToTicks(rapid_joint_goal(2));
  rapid_packet(7) = angleToTicks(rapid_joint_goal(3));
  
  pp.write(MOVE_SERV_ID, rapid_packet);
  previous_ball_locations = zeros(3, 2, 'single');
  
  while true
      current_image = snapshot(cam);
      packet = zeros(15, 1, 'single');
      pp.write(STATUS_SERV_ID, packet);
      pause(0.003);
      returnPacket = pp.read(STATUS_SERV_ID); % gets current position data
      
      [green_loc, blue_loc, yellow_loc] = findObjs(current_image, T_cam_to_checker, cameraParams);
      % BALL LOCATIONS ALWAYS USES GREEN BLUE YELLOW AS ITS ORDER
      ball_locations = [green_loc; blue_loc; yellow_loc];
      ball_locations = getCheckerboardToRobot(ball_locations);
      
      
      % if my target has changed by some meaningful amount, update the
      % trajectory coefs and decrease total step time by .5 seconds
      if outsideOfTolerances(previous_ball_locations, ball_locations)
        disp('Recalculating coefficients. . . ');
        green_goal =  ikin([ball_locations(1, 1) ball_locations(1, 2) 0]);
        blue_goal = ikin([ball_locations(2, 1) ball_locations(2, 2) 0]);
        yellow_goal = ikin([ball_locations(3, 1) ball_locations(3, 2) 0]);
        
        base_coef = trajectory(toc, toc+total_move_step-0.5, 0, 0, ticksToAngle(returnPacket(1)), green_goal(1));
        elbow_coef = trajectory(toc, toc+total_move_step-0.5, 0, 0, ticksToAngle(returnPacket(2)), green_goal(2));
        wrist_coef = trajectory(toc, toc+total_move_step-0.5, 0, 0, ticksToAngle(returnPacket(3)), green_goal(3));
      else
          move1_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
          move1_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
          move1_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
          
          pp.write(MOVE_SERV_ID, move1_packet);
          
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
