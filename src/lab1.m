%%
% RBE3001 - Laboratory 1 
% 
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
% 
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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

  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');
  pid_config_packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
  posData = [];
  line = animatedline;
  axis([0, 10, 400, 620]);
  tic
  actualPosition = 0;
  packet(1) = 600;
  packet(4) = 600;
  packet(7) = 200;
  pid_config_packet(1) = 0.00295; % base kp DO NOT CHANGE THESE PLEASE PLEASE PLEASE
  pid_config_packet(2) = 0.00010; % base ki
  pid_config_packet(3) = 0.00002; % base kd
  pid_config_packet(4) = 0.01% elbow kp DO NOT CHANGE THESE PLEASE PLEASE PLEASE
  pid_config_packet(5) = 0.00002; % elbow ki
  pid_config_packet(6) = 0.00005; % elbow kd
  pid_config_packet(7) = 0.00400; % wrist kp DO NOT CHANGE THESE PLEASE PLEASE PLEASE
  pid_config_packet(8) = 0.00007; % wrist ki
  pid_config_packet(9) = 0.00002; % wrist kd
  pp.write(PID_CONFIG_SERV_ID, pid_config_packet);
  pp.write(MOVE_SERV_ID, packet); 

  while true
      packet = zeros(15, 1, 'single');
      pp.write(STATUS_SERV_ID, packet);

      pause(0.003); % Minimum amount of time required between write and read

      %pp.read reads a returned 15 float backet from the nucleo.
      returnPacket = pp.read(STATUS_SERV_ID);
      actualPosition = returnPacket(1);
      posData = [posData ; returnPacket(1) returnPacket(2) returnPacket(3)];
      xlabel('Elapsed Time (sec)');
      ylabel('Encoder Value');

      addpoints(line, toc, double(returnPacket(1)))

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
      pause(0.1) %timeit(returnPacket) !FIXME why is this needed?
      


      csvwrite('positionData.csv', posData);
  end 
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
