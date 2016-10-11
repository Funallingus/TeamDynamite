fprintf('Train system...');
close all
initalize_system
fprintf('Done\n');
%%calibrate camera and find origin%%
[frame, depthIm, time, meta] = capture_frame(colorVid, depthVid);
[feducialBox, feducialCentroid ] = find_fiducial(frame, depthIm);


speedCalibration = (feducialBox(2, 1) - feducialBox(1,1))/50;
%%calculate origin

%%
%capture image
[frame, depthIm, time, meta] = capture_frame(colorVid, depthVid);


%%
%find dominos
[domino, boxDimensions, match, pose] = edge_detection(frame, depthIm, model,...
                    referenceLibrary, compositeLibrary, dice, feducialCentroid);

%

%%
%%%track domino%%%
knownLocation = [];
for i = 1 : size(boxDimensions, 2);
    knownLocation = [knownLocation; boxDimensions{i}];

    
end
previousImage = frame;
dominoLocationHistory = [];

%create a figure
fig = figure(30);
fig.Position = [10, 10, 1000, 700];
%figure('Position', [10, 10, 1000, 700]); %sets the figure window size for all im
set(gca,'units','pixels');
sz = size(previousImage);
cim = image(...
    [1 sz(2)],...
    [1 sz(1)],...
    zeros(sz(1), sz(2), 1),...
    'CDataMapping', 'scaled'...
);
colormap gray;
axis image;

set(cim, 'cdata', previousImage);
drawnow;

%%filter through images, update figure

[numDomino, points] = size(knownLocation);
%result = 0;
result = [];
allLoc = [];
aveDist = [];
count = 0;
for j = 1:numDomino
    result(j) = 0;
end

previousTime = time;
j = 1;
while 1
%for i = 1:9
   %do the tracking thing 
   [currentImage, depthIm, time, meta] = capture_frame(colorVid, depthVid);
   
   [newLocation] = track_image(knownLocation,previousImage,currentImage);
   
   %update figure with current image
   figure(30);
   set(cim, 'cdata', currentImage);
   drawnow;
   %place a dot at all previous tracked locations
   %hold on;

   incrTime = 0;
   for i = 1:numDomino
        if result(i) ~= 0
            delete(result(i));
        end
        x1 = newLocation(i,1);
        y1 = newLocation(i, 2);
        x2 = newLocation(i,1) + newLocation(i, 3);
        y2 = newLocation(i, 2) - newLocation(i, 4);
        width = newLocation(i,3);
        height = newLocation(i,4);
        hold on; resultDot = scatter(x1,y1,'filled'); hold off;
        if i == 1
            x = [x1, x2, x2, x1, x1];
            y = [y1, y1, y2, y2, y1];
        else
            x = [x1-width/2, x2, x2, x1-width/2, x1-width/2];
            y = [y1+height/2, y1+height/2, y2, y2, y1+height/2];
        end
        hold on; result(i) = plot(x, y, 'LineWidth', 2, 'Color', 'k'); hold off;
        distance = sqrt((knownLocation(i, 1) - (newLocation(i, 1)))^2 + ((knownLocation(i, 2) - (newLocation(i, 2)))^2)); %change in difference
        speed = distance/(time - previousTime);
        %fprintf('Speed: %g pixels/s, distance: %g \n', speed, distance);
        drawnow; 
   end
   
   if j == 0
       allLoc = newLocation;
       
       %incrTime = time-previousTime;
   elseif mod(j,6) == 0
       count = count+1;
       %incrTime = incrTime + time - previousTime;
       allLoc = [allLoc; newLocation];
   elseif j == 20
       j = 0;
       incrTime = time - previousTime;
       for i = 1:numDomino
           sumXPix = 0;
           sumYPix = 0;
           for k = 0 : count - 1
               sumXPix = sumXPix + allLoc(i+k*numDomino, 1);
               sumYPix = sumYPix + allLoc(i+k*numDomino, 2);
               %dist = sqrt(sumXPix^2 + sumYPix^2);

           end
           aveXPix = sumXPix/count;
           aveYPix = sumYPix/count;
           dist = sqrt(aveXPix^2 + aveYPix^2);
           cel = dist/incrTime;
        fprintf('Domino %g velocity: %g pixels/s; %g mm/s; dist: %g mm, time: %g s\n', i, dist/incrTime, ...
                        dist/incrTime/speedCalibration, dist/speedCalibration, incrTime);
        allLoc(i) = 0;
       end
   previousTime = time;
   count = 0;
   end
   %draw box around last tracked location
%    x1 = newLocation(numDomino ,1);
%    x2 = newLocation(numDomino,1) + newLocation(numDomino, 3);
%    y2 = newLocation(numDomino, 2) - newLocation(numDomino, 4);
%    y1 = newLocation(numDomino, 2);
%    x = [x1, x2, x2, x1, x1];
%    y = [y1, y1, y2, y2, y1];
%    plot(x, y, 'LineWidth', 2); hold off;
   
   %reinitialize
   dominoLocationHistory = [dominoLocationHistory; newLocation];
   knownLocation = newLocation;
   previousImage = currentImage;
   %previousTime = time;
   j = j + 1;
end
