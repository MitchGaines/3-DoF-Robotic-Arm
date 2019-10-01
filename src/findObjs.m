function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig, T_cam_to_checker, cameraParams)
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
%   Ideally, this function should return:
%   IMDETECTEDOBJS - a binarized image showing the location of the
%   segmented objects of interest.
%   
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
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
%imshow(im)
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

imDetectedGreen = insertObjectAnnotation(im, 'rectangle', green_boxes, 'green');
imDetectedYellow = insertObjectAnnotation(im, 'rectangle', yellow_boxes, 'yellow');
imDetectedBlue = insertObjectAnnotation(im, 'rectangle', blue_boxes, 'blue');

imshow(imDetectedGreen)
imshow(imDetectedYellow)
% imshow(imDetectedBlue)

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
fprintf('Measured diameter of one green ball = %0.2f mm\n', diameterInMillimeters);


blue_box1 = double(blue_boxes(1, :));
imagePoints1 = [blue_box1(1:2); ...
                blue_box1(1) + blue_box1(3), blue_box1(2)];
            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured diameter of one blue ball = %0.2f mm\n', diameterInMillimeters);


yellow_box1 = double(yellow_boxes(1, :));
imagePoints1 = [yellow_box1(1:2); ...
                yellow_box1(1) + yellow_box1(3), yellow_box1(2)];
            
worldPoints1 = pointsToWorld(cameraParams, R, t, imagePoints1);

d = worldPoints1(2, :) - worldPoints1(1, :);
diameterInMillimeters = hypot(d(1), d(2));
fprintf('Measured diameter of one yellow ball = %0.2f mm\n', diameterInMillimeters);

% stats = regionprops('table',yellow_mask,'Centroid','MajorAxisLength','MinorAxisLength','Area')
% 
% centers = stats.Centroid
% diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
% radii = diameters/2;
% 
% hold on
% viscircles(centers,radii);
% hold off



% You can easily convert image pixel coordinates to 3D coordinates (expressed in the
% checkerboard reference frame) using the following transformations:

R = T_cam_to_checker(1:3,1:3);
t = T_cam_to_checker(1:3,4);
% worldPoints = pointsToWorld(cameraParams, R, t, YOUR_PIXEL_VALUES);

% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)
end