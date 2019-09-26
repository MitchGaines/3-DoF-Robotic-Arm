
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
  
  move1_pos_goal = [175 -200 0];
  move2_pos_goal = [175 200 0];
  move3_pos_goal = [125 0 400];
  
  move1_joint_goal = ikin(move1_pos_goal); % this specifies the target joint angles
  move2_joint_goal = ikin(move2_pos_goal); % this specifies the target joint angles
  move3_joint_goal = ikin(move3_pos_goal);
  
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
  
  velocity = [0;0;0];
  while true
      packet = zeros(15, 1, 'single');
      pp.write(STATUS_SERV_ID, packet);
      pause(0.003); % Minimum amount of time required between write and read
      returnPacket = pp.read(STATUS_SERV_ID);   
      
      jointAngles = [ticksToAngle(returnPacket(1)) ticksToAngle(returnPacket(2)) ticksToAngle(returnPacket(3))];
      ef_velocity = fwkinJcb(jointAngles, velocity);
      
      plotArm(jointAngles, ef_velocity);
      J = jacob0(jointAngles);
      
      Jvd = det(J(1:3, : ));
      
       if -.0002 < Jvd && Jvd < .0002
         text(0,0,0,'Singularity!','HorizontalAlignment','left','FontSize',8);
         error('Singularity hit. Program terminating.')
         break;
       end
      
      switch current_move
          case 1
            move1_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move1_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move1_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move1_packet);
            pause(0.003);
            move_return = pp.read(MOVE_SERV_ID);
      
            velocity = [ticksToAngle(move_return(2)); ticksToAngle(move_return(5)); ticksToAngle(move_return(8))];
            
            
            if (jointAngles(1) < move1_joint_goal(1)+joint_tolerance && jointAngles(1) > move1_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move1_joint_goal(2)+joint_tolerance && jointAngles(2) > move1_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move1_joint_goal(3)+joint_tolerance && jointAngles(3) > move1_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(1), move2_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(2), move2_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move1_joint_goal(3), move2_joint_goal(3));
               
               pause(1);
               current_move = 2;
            end

          case 2  
            move2_packet(1) = angleToTicks(base_coef(1) + base_coef(2)*(toc+step) + base_coef(3)*((toc+step)^2)  + base_coef(4)*((toc+step)^3));
            move2_packet(4) = angleToTicks(elbow_coef(1) + elbow_coef(2)*(toc+step) + elbow_coef(3)*((toc+step)^2)  + elbow_coef(4)*((toc+step)^3));
            move2_packet(7) = angleToTicks(wrist_coef(1) + wrist_coef(2)*(toc+step) + wrist_coef(3)*((toc+step)^2)  + wrist_coef(4)*((toc+step)^3));
            
            pp.write(MOVE_SERV_ID, move2_packet); 
            pause(0.003);
            move_return = pp.read(MOVE_SERV_ID);
      
            velocity = [ticksToAngle(move_return(2)); ticksToAngle(move_return(5)); ticksToAngle(move_return(8))];
            
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
            pause(0.003);
            move_return = pp.read(MOVE_SERV_ID);
      
            velocity = [ticksToAngle(move_return(2)); ticksToAngle(move_return(5)); ticksToAngle(move_return(8))];
            
            if (jointAngles(1) < move3_joint_goal(1)+joint_tolerance && jointAngles(1) > move3_joint_goal(1)-joint_tolerance) &&  (jointAngles(2) < move3_joint_goal(2)+joint_tolerance && jointAngles(2) > move3_joint_goal(2)-joint_tolerance) && (jointAngles(3) < move3_joint_goal(3)+joint_tolerance && jointAngles(3) > move3_joint_goal(3)-joint_tolerance)
               base_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(1), move1_joint_goal(1));
               elbow_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(2), move1_joint_goal(2));
               wrist_coef = trajectory(toc, toc+total_move_step, 0, 0, move3_joint_goal(3), move1_joint_goal(3));
               
               pause(1);
               current_move = 1;
            end
      end  
      
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
  end 
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
