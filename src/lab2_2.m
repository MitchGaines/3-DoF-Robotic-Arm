
clear
clear java
clear classes;
clf

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

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs); 
try
  MOVE_SERV_ID = 01;
  PID_CONFIG_SERV_ID = 02;
  STATUS_SERV_ID = 03; 

  DEBUG   = false;          % enables/disables debug prints

  figure(1)
  xlabel('Elapsed Time (sec)');
  ylabel('Angle Value');
  line1 = animatedline('Color', 'r');
  line2 = animatedline('Color', 'g');
  line3 = animatedline('Color', 'b');
  axis([0, 1, 0, 150]);
  
  figure(3)
  line4 = animatedline('Color', 'b');
  xlabel('X position (meters)');
  ylabel('Z position (meters)');
  axis([0, 0.4, -0.1, 0.3])
  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  
  % packet sizes up to 64 bytes.
  current_move = 1;
  joint_tolerance = 5;
  step = 0.0005;
  total_move_step = 7;
  move1_packet = zeros(15, 1, 'single');
  move2_packet = zeros(15, 1, 'single');
  move3_packet = zeros(15, 1, 'single');
  move4_packet = zeros(15, 1, 'single');
  move5_packet = zeros(15, 1, 'single');
  
  move1_pos_goal = [175 -200 0];
  move2_pos_goal = [175 200 0];
  move3_pos_goal = [100 200 400];
  move4_pos_goal = [175 0 700];
  move5_pos_goal = [100 -200 400];
  
  move1_joint_goal = ikin(move1_pos_goal); % this specifies the target joint angles
  move2_joint_goal = ikin(move2_pos_goal); % this specifies the target joint angles
  move3_joint_goal = ikin(move3_pos_goal);
  move4_joint_goal = ikin(move4_pos_goal);
  move5_joint_goal = ikin(move5_pos_goal);
  
  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
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
  
  move1_packet(1) = angleToTicks(move1_joint_goal(1));
  move1_packet(4) = angleToTicks(move1_joint_goal(2));
  move1_packet(7) = angleToTicks(move1_joint_goal(3));
  
  pp.write(MOVE_SERV_ID, move1_packet); 

  current_move = 2; % since we just went to our first position, lets calculate to go from 1->2 and immediately step into move 2
  base_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(1), move2_joint_goal(1));
  elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(2), move2_joint_goal(2));
  wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(3), move2_joint_goal(3));

  while true
      packet = zeros(15, 1, 'single');
      pp.write(STATUS_SERV_ID, packet);
      
      pause(0.003); % Minimum amount of time required between write and read

      returnPacket = pp.read(STATUS_SERV_ID);
            
      jointAngles = [ticksToAngle(returnPacket(1)) ticksToAngle(returnPacket(2)) ticksToAngle(returnPacket(3))];
      
      switch current_move
          case 1
            move1_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move1_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move1_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move1_packet); 

            if (jointAngles(1) < move1_joint_goal(1)+joint_tolerance && jointAngles(1) > move1_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move1_joint_goal(2)+joint_tolerance && jointAngles(2) > move1_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move1_joint_goal(3)+joint_tolerance && jointAngles(3) > move1_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(1), move2_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(2), move2_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(3), move2_joint_goal(3));
               
               pause(1);
               current_move = 2;
            end

          case 2  
            calculated_points = [];
            calculated_points(1) = base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3);
            move2_packet(1) = angleToTicks(calculated_points(1));
            calculated_points(2) = elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3);
            move2_packet(4) = angleToTicks(calculated_points(2));
            calculated_points(3) = wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3);
            move2_packet(7) = angleToTicks(calculated_points(3));
            
            pp.write(MOVE_SERV_ID, move2_packet); 
            
            if (jointAngles(1) < move2_joint_goal(1)+joint_tolerance && jointAngles(1) > move2_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move2_joint_goal(2)+joint_tolerance && jointAngles(2) > move2_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move2_joint_goal(3)+joint_tolerance && jointAngles(3) > move2_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move2_joint_goal(1), move3_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move2_joint_goal(2), move3_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move2_joint_goal(3), move3_joint_goal(3));
               
               pause(1);
               current_move = 3;
            end
          case 3  
            move3_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move3_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move3_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move3_packet); 
            
            if (jointAngles(1) < move3_joint_goal(1)+joint_tolerance && jointAngles(1) > move3_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move3_joint_goal(2)+joint_tolerance && jointAngles(2) > move3_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move3_joint_goal(3)+joint_tolerance && jointAngles(3) > move3_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(1), move4_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(2), move4_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(3), move4_joint_goal(3));
               
               pause(1);
               current_move = 4;
            end
         case 4
            move4_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move4_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move4_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move4_packet); 
            
            if (jointAngles(1) < move4_joint_goal(1)+joint_tolerance && jointAngles(1) > move4_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move4_joint_goal(2)+joint_tolerance && jointAngles(2) > move4_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move4_joint_goal(3)+joint_tolerance && jointAngles(3) > move4_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move4_joint_goal(1), move5_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move4_joint_goal(2), move5_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move4_joint_goal(3), move5_joint_goal(3));
               
               pause(1);
               current_move = 5;
            end
         case 5
            move5_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move5_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move5_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move5_packet); 
            
            if (jointAngles(1) < move5_joint_goal(1)+joint_tolerance && jointAngles(1) > move5_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move5_joint_goal(2)+joint_tolerance && jointAngles(2) > move5_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move5_joint_goal(3)+joint_tolerance && jointAngles(3) > move5_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move5_joint_goal(1), move1_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move5_joint_goal(2), move1_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move5_joint_goal(3), move1_joint_goal(3));
               
               pause(1);
               current_move = 1;
            end
      end
      
      figure(1)
      xlim([0, toc])
      addpoints(line1, toc, double(jointAngles(1)))
      addpoints(line2, toc, double(jointAngles(2)))
      addpoints(line3, toc, double(jointAngles(3)))
      
%       figure(2)
%       plotArm(jointAngles);
%       
%       figure(3)
%       fwkinEF = fwkin3001(jointAngles);
%       addpoints(line4, double(fwkinEF(1)), double(fwkinEF(3)))
      
      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end

      for x = 0:3
          packet((x*3)+1)=0.1;
          packet((x*3)+2)=0;
          packet((x*3)+3)=0;
      end
      toc
        
      % csvwrite('EFData.csv', fwkinEF);
      % pause(0.1)
  end 
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
