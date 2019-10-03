function [ converted_locations ] = getCheckerboardToRobot(locations)
% getCheckerBoardToRobot converts locations in the checkerboard's 
% reference frame into locations in the robot's reference frame
    converted_locations = locations;
    for row=1:size(locations, 1)
        converted_locations(row,1) = locations(row, 1) + 238.8;
        converted_locations(row,2) = locations(row, 2) - 126.4;
    end
end

