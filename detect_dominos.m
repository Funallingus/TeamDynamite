%LAB2init;
close all
initalize_system;
%captureJpeg('Dice library/dice 66.jpg', colorVid, depthVid);
%stop([colorVid depthVid]);

%image = imread('SIFT_TEST/cluttered.jpeg');
[frame, depthIm, time, meta] = capture_frame(colorVid, depthVid);

[boxPolygon, centroid] = find_fiducial(frame, depthIm);
[domino, dominoBoxDimensions, dominoMatch, dominoPose] = edge_detection(frame, depthIm,...
            model, referenceLibrary, compositeLibrary, dice, centroid);
