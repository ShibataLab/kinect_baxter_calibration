% compute_projection.m: Program to spatially align motion capture and Kinect Readings
% and apply absolute orientation algorithm for calibration
% Author: Nishanth Koganti
% Date: 2016/6/15

% TODO:
% 1) Implement in python

function compute_projection(fileName, mode)

close all;
fontSize = 12;
markerSize = 25;

if mode == 0
    % Kinect Calibration Mode
    [~,kinectData] = parseKinect(fileName);
    [~,mocapData] = parseMocap(sprintf('%s.trc',fileName));

    kinectData = kinectData(30:end,:);

    nMocap = size(mocapData,1);
    nKinect = size(kinectData,1);
    nMarkers = (size(mocapData,2) - 2)/3;

    mocapT = mocapData(:,2);
    kinectT = kinectData(:,2);

    indices = 0:40;
    errs = zeros(1,length(indices));
    minErr = 1.0; minOffset = 0; minMocapPos = [];
    minR = []; minT = []; minc = 0.0; minKOut = [];


    for freq = indices
        mocapInd = zeros(nKinect,1);

        for j = 1:nKinect
            tRef = kinectT(j);
            [~,ind] = min((tRef - mocapT).^2);
            if ind - freq < 1
                mocapInd(j) = 1;
            elseif ind - freq > nMocap
                mocapInd(j) = nMocap;
            else
                mocapInd(j) = ind - freq;
            end
        end

        kinPos = kinectData(:,3:5);
        mocapPos = zeros(nKinect,3);

        for j = 1:nMarkers
            mocapPos(:,1) = mocapPos(:,1) + mocapData(mocapInd,3+(j-1)*3);
            mocapPos(:,2) = mocapPos(:,2) + mocapData(mocapInd,4+(j-1)*3);
            mocapPos(:,3) = mocapPos(:,3) + mocapData(mocapInd,5+(j-1)*3);
        end
        mocapPos = mocapPos./nMarkers;

        [R,T,c,err,kOut] = absoluteOrientationSVD(kinPos',mocapPos',0);

        errs(freq+1) = err;
        if err < minErr
            minR = R; minT = T; minc = c; minKOut = kOut;
            minErr = err; minOffset = freq; minMocapPos = mocapPos;
        end
    end

    kinOut = minKOut';
    transMatrix = [minc*minR; 0 0 0];
    transMatrix = [transMatrix [minT; 1]];
    dlmwrite(sprintf('%s.cal',fileName), transMatrix);
    dlmwrite(sprintf('%s.offset',fileName), minOffset);
    fprintf('Error: %f, Offset: %d\n',minErr,minOffset);

    figure;
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Kinect Calibration',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    axis([-0.1 0.6 -0.1 0.6 1.0 1.8]);
    view([45 45]);

    for i = 1:nKinect
        pl1 = plot3(minMocapPos(i,1), minMocapPos(i,2), minMocapPos(i,3), '.b', 'MarkerSize', markerSize);
        pl2 = plot3(kinOut(i,1), kinOut(i,2), kinOut(i,3), '.r', 'MarkerSize', markerSize);

        drawnow;
        pause(0.1);
        delete(pl1);
        delete(pl2);
    end

    plot3(minMocapPos(:,1), minMocapPos(:,2), minMocapPos(:,3), '.b', 'MarkerSize', markerSize);
    plot3(kinOut(:,1), kinOut(:,2), kinOut(:,3), '.r', 'MarkerSize', markerSize);
    hold off;

    figure;
    plot(indices,errs,'.-','LineWidth',2);

    figure;
    hold on;
    plot(1:nKinect, minMocapPos(:,1), 1:nKinect, kinOut(:,1), 'LineWidth',2);
    legend('Mocap','Kinect');
    hold off;

elseif mode == 1
    % Baxter Calibration Mode
    [~,baxterData] = parseBaxter(sprintf('%sEE',fileName));
    [~,mocapData] = parseMocap(sprintf('%s.trc',fileName));

    freq = 1;

    baxterData = baxterData(100:freq:end,:);

    mocapT = mocapData(:,2);
    baxterT = baxterData(:,1);

    nBaxter = size(baxterData,1);
    nMarkers = (size(mocapData,2) - 2)/3;

    mocapPos = zeros(nBaxter,3);
    baxterPos = baxterData(:,2:4);

    mocapInd = zeros(nBaxter,1);
    for j = 1:nBaxter
        tRef = baxterT(j);
        [~,ind] = min((tRef - mocapT).^2);
        mocapInd(j) = ind;
    end

    for j = 1:nMarkers
        mocapPos(:,1) = mocapPos(:,1) + mocapData(mocapInd,3+(j-1)*3);
        mocapPos(:,2) = mocapPos(:,2) + mocapData(mocapInd,4+(j-1)*3);
        mocapPos(:,3) = mocapPos(:,3) + mocapData(mocapInd,5+(j-1)*3);
    end
    mocapPos = mocapPos./nMarkers;

    [R,T,c,err,baxterOut] = absoluteOrientationSVD(baxterPos',mocapPos',1);

    baxterOut = baxterOut';
    transMatrix = [c*R; 0 0 0];
    transMatrix = [transMatrix [T; 1]];
    dlmwrite(sprintf('%s.cal',fileName), transMatrix);
    dlmwrite(sprintf('%s.offset',fileName), 0);
    fprintf('Error: %f, Offset: %d\n',err,0);

    figure;
    hold on;
    xlabel('X [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    zlabel('Z [m]', 'FontSize', fontSize, 'FontWeight', 'bold');
    title('Baxter Calibration',  'FontSize', fontSize, 'FontWeight', 'bold');
    set(gca, 'FontSize', fontSize, 'FontWeight', 'bold');
    % axis([-0.1 0.6 -0.1 0.6 1.0 1.8]);
    view([45 45]);

    plot3(mocapPos(:,1), mocapPos(:,2), mocapPos(:,3), '.b', 'MarkerSize', markerSize);
    plot3(baxterOut(:,1), baxterOut(:,2), baxterOut(:,3), '.r', 'MarkerSize', markerSize);
    hold off;

    figure;
    hold on;
    plot(1:nBaxter, mocapPos(:,1), 1:nBaxter, baxterOut(:,1), 'LineWidth',2);
    legend('Mocap','Kinect');
    hold off;
end

pause;
close all;
