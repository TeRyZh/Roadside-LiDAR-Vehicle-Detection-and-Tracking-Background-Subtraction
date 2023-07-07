%% Load raw LiDAR Data

myDir = 'E:\French@JoyceLiDAR\LiDAR'; %gets directory
addpath(myDir);

myFiles = dir(fullfile(myDir,'*.pcap')); %gets all wav files in struct

%% load forground
load("GMM_French@Joyce1110.mat");
%% Save to GIF
xlimits = [-100 100];
ylimits = [-100 100];
zlimits = [-10 0];

ax = axes('Parent',uipanel);
axis([-100 100 -100 100]);
SegmentationMinDistance = 1.2;  % lateral is usually 1 meters, following cars usually 1.8 ft
minimum_cluster_pts = 50;
start_Frame = 1;
total_frames = 1000;

frame_num = 0;

for k = 1:length(myFiles)
  baseFileName = myFiles(k).name;
  fullFileName = fullfile(myDir, baseFileName);
  veloReader = velodyneFileReader(fullFileName,'VLS128');

  while(hasFrame(veloReader))

      ptCloudObj = readFrame(veloReader);

      frame_num = frame_num + 1;

      if frame_num < start_Frame

          continue;
      end

      if frame_num > start_Frame + total_frames

          break;
          
      end

    fg_LiDAR = lidarData{frame_num};

    fg_LiDAR = pcdenoise(fg_LiDAR,'NumNeighbors', 4);

    [labels,numClusters] = pcsegdist(fg_LiDAR, SegmentationMinDistance);

    % Select each cluster and fit a cuboid to each cluster.
    bboxes = [];
    for num = 1:numClusters
        labelIdx = (labels == num);
        
        % Ignore cluster that has points less than 200 points.
        if sum(labelIdx,'all') < minimum_cluster_pts
            continue;
        end

        pcSeg = select(fg_LiDAR, labelIdx);

        try
            mdl = pcfitcuboid(pcSeg);
            bboxes = [bboxes; mdl.Parameters];
        catch
            continue;
        end
    end

    pcshowpair(ptCloudObj, fg_LiDAR, 'Parent', ax);
    showShape('cuboid',bboxes,'Parent',ax,'Opacity',0.1,...
          'Color','yellow','LineWidth',0.5);

    view(2);
    axis([-100 100 -100 100]);
    frame = getframe(gcf);
    img =  frame2im(frame);
    [img,cmap] = rgb2ind(img,256);

    % Write to the GIF File 
    if frame_num == start_Frame
        imwrite(img,cmap,'GMM_FrenchJoyce_animation.gif','gif','LoopCount',Inf,'DelayTime',0.05);
    else
        imwrite(img,cmap,'GMM_FrenchJoyce_animation.gif','gif','WriteMode','append','DelayTime',0.05);
    end

  end

end