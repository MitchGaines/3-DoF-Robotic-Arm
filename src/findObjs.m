function [green_world_location, blue_world_location, yellow_world_location] = findObjs(imOrig, T_cam_to_checker, cameraParams)
% function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)
% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.
%
% Note: this function contains several un-implemented sections - it only
% provides a skeleton that you can use as reference in the development of
% your image processing pipeline. Feel free to edit as needed (or, feel
% free to toss it away and implement your own function).
%
%   Usage
%   -----
%   [IMDETECTEDOBJS, ROBOTFRAMEPOSE] = findObjs(IMORIG, TCHECKER2ROBOT, TCAM2CHECKER, CAMERAPARAMS)
%
%   Inputs
%   ------
%   IMORIG - an RGB image showing the robot's workspace (capture from a CAM
%   object).
%
%   TCHECKER2ROBOT - the homogeneous transformation matrix between the
%   checkered board and the reference frame at the base of the robot.
%
%   TCAM2CHECKER - the homogeneous transformation matrix between the camera
%   reference frame and the checkered board (you can calculate this using
%   the GETCAMTOCHECKERBOARD function, provided separately).
%
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
%
%   Outputs
%   -------
%   green_world_location:
%   the location of a green detection  
%
%   blue_world_location:
%   the location of a blue detection
%
%   yellow_world_location:
%   the location of a yellow detection
%
%   Authors
%   -------
%   Nathaniel Dennler  <nsdennler@wpi.edu>
%   Sean O'Neil        <stoneil@wpi.edu> 
%   Loris Fichera      <lfichera@wpi.edu>
%
%   Latest Revision
%   ---------------
%   2/12/2019


%%  1. First things first - undistort the image using the camera parameters
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');

%%  2. Segment the image to find the objects of interest.
blured_image = imgaussfilt(im, 4);

green_mask = createGreenMask(blured_image);
yellow_mask = createYellowMask(blured_image);
blue_mask = createBlueMask(blured_image);

blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
    'CentroidOutputPort', false,...
    'BoundingBoxOutputPort', true,...
    'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);

[green_areas, green_boxes] = step(blobAnalysis, green_mask);
[yellow_areas, yellow_boxes] = step(blobAnalysis, yellow_mask);
[blue_areas, blue_boxes] = step(blobAnalysis, blue_mask);

green_boxes = double(green_boxes(:, :));
yellow_boxes = double(yellow_boxes(:, :));
blue_boxes = double(blue_boxes(:, :));

if isempty(green_boxes)
    green_boxes = [-100 -100 10 10];
end
if isempty(yellow_boxes)
    yellow_boxes = [-100 -100 10 10];
end
if isempty(blue_boxes)
    blue_boxes = [-100 -100 10 10];
end

detections = insertObjectAnnotation(im, 'rectangle', [green_boxes(1, :); yellow_boxes(1, :); blue_boxes(1, :)], ['Ball'], 'LineWidth', 3, 'Color', {'green', 'yellow', 'blue'},'TextColor','black');
imshow(detections)
%imshow(imDetectedGreen)
%imshow(imDetectedYellow)
%imshow(imDetectedBlue)

R = T_cam_to_checker(1:3,1:3);
t = T_cam_to_checker(1:3,4);

green_boxes = green_boxes + [newOrigin, 0, 0];
yellow_boxes = yellow_boxes + [newOrigin, 0, 0];
blue_boxes = blue_boxes + [newOrigin, 0, 0];

green_box1 = double(green_boxes(1, :));
imagePoints1 = [green_box1(1:2); ...
                green_box1(1) + green_box1(3), green_box1(2)];
            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
green_world_location = pointsToWorld(cameraParams, R, t, [(green_boxes(1)+green_boxes(3)/2) (green_boxes(2)+green_boxes(4)/2)])
% fprintf('Measured diameter of one green ball = %0.2f mm\n', diameterInMillimeters);

blue_box1 = double(blue_boxes(1, :));
imagePoints1 = [blue_box1(1:2); ...
                blue_box1(1) + blue_box1(3), blue_box1(2)];
            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
blue_world_location = pointsToWorld(cameraParams, R, t, [(blue_boxes(1)+blue_boxes(3)/2) (blue_boxes(2)+blue_boxes(4)/2)])
% fprintf('Measured diameter of one blue ball = %0.2f mm\n', diameterInMillimeters);

yellow_box1 = double(yellow_boxes(1, :));
imagePoints1 = [yellow_box1(1:2); ...
                yellow_box1(1) + yellow_box1(3), yellow_box1(2)];
            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
yellow_world_location = pointsToWorld(cameraParams, R, t, [(yellow_boxes(1)+yellow_boxes(3)/2) (yellow_boxes(2)+yellow_boxes(4)/2)])
% fprintf('Measured diameter of one yellow ball = %0.2f mm\n', diameterInMillimeters);

% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)
end