
initalize_system
%%calibrate camera and find origin%%
[frame, time, meta] = capture_frame(colorVid, depthVid);
[feducialBox, feducialCentroid ] = find_fiducial(frame);
%%
%capture image
[frame, time, meta] = capture_frame(colorVid, depthVid);

while 1
    %%
    %find dominos once (~4 seconds)
    [domino, dominoCentroid_x, dominoCentroid_y, dominoMatch] = edge_detection(frame, model, referenceLibrary);

    %%
    %%%track domino%%%
    %track for ~15 seconds

end