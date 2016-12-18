close all;

%% Parameters

% options
vidFile = '';              % Use video file instead of camera as input
markerLength = 0.1;        % Marker side length (in meters). Needed for correct scale in camera pose
dictionaryId = '6x6_250';  % Dictionary id
showRejected = false;      % Show rejected candidates too
estimatePose = false;      % Wheather to estimate pose or not
if estimatePose
    % Camera intrinsic parameters. Needed for camera pose
    load camera_parameters.mat camMatrix distCoeffs
    %camMatrix = eye(3);
    %distCoeffs = zeros(1,5);
else
    camMatrix = [];
    distCoeffs = [];
end

% marker detector parameters
detectorParams = struct();
if false
    detectorParams.nMarkers = 1024;
    detectorParams.adaptiveThreshWinSizeMax = 21;
    detectorParams.adaptiveThreshConstant = 7;
    detectorParams.minMarkerPerimeterRate = 0.03;
    detectorParams.maxMarkerPerimeterRate = 4.0;
    detectorParams.polygonalApproxAccuracyRate = 0.05;
    detectorParams.minCornerDistanceRate = 10.0;
    detectorParams.minDistanceToBorder = 3;
    detectorParams.minMarkerDistanceRate = 10.0;
    detectorParams.cornerRefinementWinSize = 5;
    detectorParams.cornerRefinementMaxIterations = 30;
    detectorParams.cornerRefinementMinAccuracy = 0.1;
    detectorParams.markerBorderBits = 1;
    detectorParams.perspectiveRemovePixelPerCell = 8;
    detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.13;
    detectorParams.maxErroneousBitsInBorderRate = 0.04;
end
detectorParams.doCornerRefinement = true;  % do corner refinement in markers

% dictionary
dictionary = {'Predefined', dictionaryId};

%% Input source
if ~isempty(vidFile) && exist(vidFile, 'file')
    vid = cv.VideoCapture(vidFile);
    waitTime = 1;     % 1 sec
else
    vid = cv.VideoCapture(1);
    
    waitTime = 0.01;  % 10 msec
end
if ~vid.isOpened(), error('failed to initialize VideoCapture'); end

%% Main loop
hImg = gobjects(0);
imgDetected = false; %if markers detected
pathDrawn = false; % if current path exists
cornerLoc = []; % video marker annotation container
pathLoc = []; % video path annotation container
pathI = 0; % # point on path
threshold = 20; %number of pixels to be from current goal
thresholdCost = 0.99; % angle delta to be from next point 1 = 0 degrees
% Pointers for plot annotations
plotPtr = [];
plotSt = [];
plotEnd = [];
startGoal = []; % current point on path
endGoal = []; % last point on path
tot = []; % total number of points
count = []; % current point #
gTruth = []; % path points
actual = []; % actual path taken points
setConv = false; % has pixel to mm mapping taken place
mm2pix = 0; % mm per pixel of current image
conversion = 145; % size of aruco marker side
timeThreshold = 0.05; % delta time needed to wait before sending next
                      % movement command

% Check if bluetooth already enabled
if exist('s','var') == 0
    disp('Connecting to Bluetooth...');
    s = connect_bluetooth();
    disp('Bluetooth Connected!');
    fprintf(s,sprintf('S'));
end


% Create main window to track robot
iptsetpref('ImshowAxesVisible','on');
mainWin = figure('units','normalized','outerposition',[0 0 1 1]);
img = vid.read();
hImg = imshow(img, 'XData', [0 639], 'YData', [0 479]);
title('Finding Hexapod...');
drawnow;
hold on;
tic;

% Main loop
while true
    % grab frame
    img = vid.read();
    if isempty(img), break; end

    % detect markers and estimate pose
    [corners, ids, rejected] = cv.detectMarkers(img, dictionary, ...
        'DetectorParameters',detectorParams);

    % draw results
    if ~isempty(ids) && (numel(ids) == 4) % only if 4 markers detected
        
        if ~setConv % Find mm to pixel conversion
            setConv = true;
            % Based off of one side of 4x4 array of markers
            cur31 = find(ids==31); % find marker 31
            cur203 = find(ids==203); % find marker 203
            PT1 = [corners{cur31}{3}(1) corners{cur31}{3}(2)]; %if image flipped, 31=4 203=1
            PT2 = [corners{cur203}{2}(1) corners{cur203}{2}(2)];
            
            mm2pix = conversion/pdist([PT1;PT2],'euclidean');
            
            % Display conversion
            disp(sprintf('Each pixel is %f mm',mm2pix));
            
        end
        % draw ID numbers on markers detected
        img = cv.drawDetectedMarkers(img, corners, 'IDs',ids);
        % clear last plotted marker corners
        for i = 1:numel(cornerLoc)
            delete(cornerLoc(i))
        end
        % plot new marker corners
        cornerLoc = [];
        for i=1:numel(ids)
            cornerLoc(i) = plot([corners{i}{1}(1),corners{i}{2}(1),corners{i}{3}(1),corners{i}{4}(1)],[corners{i}{1}(2),corners{i}{2}(2),corners{i}{3}(2),corners{i}{4}(2)],'ms');
        end
        
        % Find and plot the center of the marker array
        PTX = [corners{1}{1}(1);corners{2}{4}(1);corners{3}{2}(1);corners{4}{3}(1)];
        PTY = [corners{1}{1}(2);corners{2}{4}(2);corners{3}{2}(2);corners{4}{3}(2)];
        
        center = centroid(PTX,PTY);
        cornerLoc(i+1) = plot(center(1),center(2),'r.','MarkerSize',10);      
        
        
        if pathDrawn % if a current path exists
            % Determine orientation of robot using two of the Aruco Markers
            cur31 = find(ids==31);
            cur203 = find(ids==203);
            % find orientation point (point between top corner of each marker)
            PTX = [corners{cur31}{2}(1);corners{cur203}{3}(1)]; %if image flipped, 31=4 203=1
            PTY = [corners{cur31}{2}(2);corners{cur203}{3}(2)];
            orient = [mean(PTX),mean(PTY)];
            
            %find orientation vector from center point
            orientVec = orient-center;            
            
            %find vector to next point from orientation point
            nextVec = [x(count),y(count)] - orient;
            
            %find vector to next point from center point
            goalVec = [x(count),y(count)] - center;
            
            % plot guide lines from reference points/vectors found above
            cornerLoc(i+2) = plot([center(1);orient(1)],[center(2);orient(2)],'-g','LineWidth',2);
            cornerLoc(i+3) = plot([orient(1),x(count)],[orient(2),y(count)],'-m','LineWidth',2);
            cornerLoc(i+4) = plot([center(1),x(count)],[center(2),y(count)],'-b','LineWidth',2);
            
            %find magnitude of each vector
            magO = sqrt(orientVec(1)^2 + orientVec(2)^2);
            magN = sqrt(nextVec(1)^2 + nextVec(2)^2);
            magG = sqrt(goalVec(1)^2 + goalVec(2)^2);
            
            % normalize each vector
            normO = orientVec./magO;
            normN = nextVec./magN;
            normG = goalVec./magG;
            
            % take the dot product of vector to next point and orientation
            % of robot
            cost = dot(normO,normG); % cost of angle of vectors (1 is good, -1 is bad)
            
            % determine smallest angle needed to turn 
            theta = atan2d(normG(1)*normN(2)-normG(2)*normN(1),normG(1)*normN(1)+normG(2)*normN(2));
            
            % determine distance (euclidean) in pixels center of robot is
            % to current target point
            d = pdist([center;startGoal],'euclidean');       
            
            % if robot is close to current point and facing next point
            if(d < threshold && cost > thresholdCost)
                % log position of robot
                actual(:,count) = [center(1);center(2)];
                delete(plotSt);
                count = count + 1;
                % update current point to next point
                if (count <= numel(x))
                    plotSt = plot(x(count),y(count),'wh','MarkerSize',10,'MarkerFaceColor','w');
                    startGoal = [x(count),y(count)];
                    % update percentage of path completed
                    title(sprintf('Path Complete: %0.2f%%',(count/tot)*100));
                end
                % plot position of robot
                pathI = pathI + 1;
                pathLoc(pathI) = plot(center(1),center(2),'g*');
                
                if count > numel(x)%startGoal == endGoal
                    fprintf(s,sprintf('S')); % stop robot
                    % Print 100% completed
                    title(sprintf('Path Complete: %0.2f%%',((count-1)/tot)*100));
                    pause(3);
                    % Print error graph
                    nf = figure('units','normalized','outerposition',[0 0 1 1]);                                      
                    vals = [];
                    for i = 1:numel(x)
                        vals(i) = pdist([x(i) y(i);actual(1,i) actual(2,i)],'euclidean');
                    end
                    bar(vals*mm2pix/10);                    
                    title(sprintf('Mean Error: %.2f cm, Error Std: %.2f cm, Estimated Error: %.2f cm\n Max Error: %.2f cm, Min Error: %.2f cm',mean(vals)*mm2pix/10,std(vals)*mm2pix/10,mean(vals)*mm2pix/10+2*std(vals)*mm2pix/10, max(vals)*mm2pix/10, min(vals)*mm2pix/10));

                    pause; % wait for user input
                    % close error graph and continue application
                    close(nf);
                    figure(mainWin)
                    actual = [];
                    pathDrawn = false;
                    delete(plotEnd);
                    delete(plotSt);
                    delete(plotPtr);
                    title('Draw a path!');
                    for i = 1:numel(pathLoc)
                        delete(pathLoc(i))
                    end
                    pathI = 0;
                        
                end
                
            % if we are not facing the current point    
            elseif (cost < thresholdCost)
                % if maximum time elapsed from last movement command
                if(toc > timeThreshold)
                    % turn left
                    if(theta > 0)
                        fprintf(s,sprintf('L'));
                    % turn right
                    else
                        fprintf(s,sprintf('R'));
                    end
                    tic; % restart timer
                end
            % if we are not close enough to current point    
            elseif (d > threshold)
                % if maximum time elapsed from last movement command
                if(toc > timeThreshold)
                    % move forward
                    fprintf(s,sprintf('F'))
                    tic; % restart timer
                end
            end                
        end
        % Image was found   
        imgDetected = true; 
    else % Image was not found, stop the robot
        imgDetected = false;
        fprintf(s,sprintf('S')); % stop
    end

    % Update live stream
    if isempty(hImg)
        hImg = imshow(img, 'XData', [0 639], 'YData', [0 479]);
    elseif ishghandle(hImg)
        set(hImg, 'CData',img);
    else
        break;
    end
    drawnow; % update 
    
    % if robot detected and no path drawn
    if imgDetected && ~pathDrawn
        % draw a path
        [pathDrawn,plotPtr,plotSt,plotEnd,startGoal,endGoal,tot,count,x,y] = drawPath(gca);
    end
    
end
vid.release(); % release webcam

% Draw path
function [pathDrawn,plotPtr,plotSt,plotEnd,startGoal,endGoal,tot,count,x,y] = drawPath(gca,pathDrawn,plotPtr,plotSt,plotEnd,startGoal,endGoal,tot,count)
    h = imfreehand(gca); % create path drawing object
    data=get(h); % get drawn path
    xydata=get(data.Children(4)); 
    x=xydata.XData; 
    y=xydata.YData; 
    hold on;   
    h.delete(); % delete drawn path
    % Delete previous plots (if this is not the first time)
    if exist('plotPtr','var') == 1
        delete(plotPtr);
        delete(plotSt);
        delete(plotEnd);
    end
    % Plot path
    plotPtr = plot(x,y,'-r','LineWidth',3);
    % Plot current point (first point)
    plotSt = plot(x(1),y(1),'wh','MarkerSize',10,'MarkerFaceColor','w');
    % Plot last point
    plotEnd = plot(x(end),y(end),'yp','MarkerSize',10,'MarkerFaceColor','y');
    pathDrawn = true; % set flag
    % update goals
    startGoal = [x(1),y(1)];
    endGoal = [x(end),y(end)];
    tot = length(x); % total points
    count = 1; % point counter
    title('Going to start!');
end

function center = centroid(varargin)
    %CENTROID Compute centroid (center of mass) of a set of points
    %
    %   PTS = centroid(POINTS)
    %   PTS = centroid(PTX, PTY)
    %   Computes the ND-dimensional centroid of a set of points. 
    %   POINTS is an array with as many rows as the number of points, and as
    %   many columns as the number of dimensions. 
    %   PTX and PTY are two column vectors containing coordinates of the
    %   2-dimensional points.
    %   The result PTS is a row vector with Nd columns.
    %
    %   PTS = centroid(POINTS, MASS)
    %   PTS = centroid(PTX, PTY, MASS)
    %   Computes center of mass of POINTS, weighted by coefficient MASS.
    %   POINTS is a Np-by-Nd array, MASS is Np-by-1 array, and PTX and PTY are
    %   also both Np-by-1 arrays.
    %
    %   Example:
    %   pts = [2 2;6 1;6 5;2 4];
    %   centroid(pts)
    %   ans =
    %        4     3
    %
    %   See Also:
    %   points2d, polygonCentroid
    %   
    % ---------
    % Author: David Legland
    % e-mail: david.legland@grignon.inra.fr
    % created the 07/04/2003.
    % Copyright 2010 INRA - Cepia Software Platform.
    %

    %   HISTORY
    %   2009-06-22 support for 3D points
    %   2010-04-12 fix bug in weighted centroid
    %   2010-12-06 update doc


    % extract input arguments

    % use empty mass by default
    mass = [];

    if nargin==1
        % give only array of points
        pts = varargin{1};

    elseif nargin==2
        % either POINTS+MASS or PX+PY
        var = varargin{1};
        if size(var, 2)>1
            % arguments are POINTS, and MASS
            pts = var;
            mass = varargin{2};
        else
            % arguments are PX and PY
            pts = [var varargin{2}];
        end

    elseif nargin==3
        % arguments are PX, PY, and MASS
        pts = [varargin{1} varargin{2}];
        mass = varargin{3};
    end

    % compute centroid

    if isempty(mass)
        % no weight
        center = mean(pts);

    else
        % format mass to have sum equal to 1, and column format
        mass = mass(:)/sum(mass(:));

        % compute weighted centroid
        center = sum(bsxfun(@times, pts, mass), 1);
        % equivalent to:
        % center = sum(pts .* mass(:, ones(1, size(pts, 2))));
    end
end
