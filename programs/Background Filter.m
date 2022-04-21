addpath('E:\GeorgeSt_Velodyne\Albany@George0730') 
% veloReader = velodyneFileReader('DATA_20220120_161000.pcap','VLS128');
myDir = 'E:\GeorgeSt_Velodyne\Albany@George0730';
myFiles = dir(fullfile(myDir,'*.pcap')); %gets all wav files in struct

% veloReader = velodyneFileReader('D:\Dropbox\LiDAR_Data\VLP32c_0Degree162inch_FieldTest_2_ConstructionSite.pcap','VLP32C');
% veloReader = velodyneFileReader([lidar_file_name '.pcap'],'VLS128');

% use the range threshold matrix to filter out foreground LiDAR points

%%
xlimits = [-150 150];
ylimits = [-150 150];
zlimits = [-10 0];

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');

player2 = pcplayer(xlimits,ylimits,zlimits);

xlabel(player2.Axes,'X (m)');
ylabel(player2.Axes,'Y (m)');
zlabel(player2.Axes,'Z (m)');

%%
total_grid = 1800;
azimuth_unit = 360/total_grid;

roi_mask = imread("George_st_ROI.png");
xLim = xlimits(2) - xlimits(1);
yLim = ylimits(2) - ylimits(1);
zLim = zlimits(2) - zlimits(1);

load('beams_elevation.mat')
load('range_thrld_matrix.mat');


% ax = axes('Parent',uipanel);
% axis([-100 50 -50 100]);

%%

frame_num = 1;
lidarData = {};

for k = 1:length(myFiles)
    baseFileName = myFiles(k).name;
    fullFileName = fullfile(myDir, baseFileName);
    veloReader = velodyneFileReader(fullFileName,'VLS128');
    
    while(hasFrame(veloReader) && player.isOpen())
    
    %     disp(veloReader.CurrentTime);
        ptCloudObj = readFrame(veloReader);

        [ptCloudOut,~] = cropPointCloud(ptCloudObj,xlimits,ylimits,zlimits);
        
        [ptCloud_roi_filtered] = roi_filter(ptCloudOut,roi_mask);

%         view(player,ptCloud_roi_filtered.Location,ptCloud_roi_filtered.Intensity);
        
        roi_filtered_x = ptCloud_roi_filtered.Location(:,1);
        roi_filtered_y = ptCloud_roi_filtered.Location(:,2);
        roi_filtered_z = ptCloud_roi_filtered.Location(:,3);
        
        intensities = ptCloud_roi_filtered.Intensity;
        
        [azimuths, elevations, ranges] = cart2sph(roi_filtered_x,roi_filtered_y,roi_filtered_z);
        
        degree = rad2deg(azimuths);
        degree(degree<0) = degree(degree<0) + 360;
        grids_idx = mod(floor(degree/azimuth_unit) + 1, total_grid);
        grids_idx(grids_idx == 0) = total_grid -1;  % deal with 1799 + 1 grid
        grids_idx(isnan(grids_idx))=0; % LiDAR point NaN is set as 0
        
        back_idxes = zeros(size(intensities));
        
        for pnt = 1 : size(elevations,1)
            
            range = ranges(pnt);
            
            intensity = intensities(pnt);
            
            elevation = elevations(pnt);
            
            grid_idx = grids_idx(pnt);
    
            if grid_idx == 0
                
                back_idxes(pnt) = 1;
                continue;
    
            end
            
            [~, channel_num] = min(abs(beams_elevation - elevation));
            
            distance_flag = double(range) >= range_thrld_matrix(channel_num, grid_idx);
            
            if distance_flag
                
                back_idxes(pnt) = 1;
                
            end
            
        end
        
        range_filtered_x = roi_filtered_x(back_idxes == 0);
        range_filtered_y = roi_filtered_y(back_idxes == 0);
        range_filtered_z = roi_filtered_z(back_idxes == 0);
        range_filtered_intensities = intensities(back_idxes == 0);
        
        ptCloud_filtered.Location = [range_filtered_x, range_filtered_y, range_filtered_z];
        ptCloud_filtered.Intensity = range_filtered_intensities;
        
        ptCloud_obj = pointCloud(ptCloud_filtered.Location, 'Intensity', ptCloud_filtered.Intensity);
    
        ptCloud_obj = pcdenoise(ptCloud_obj); % remove noise
    
%         pcshowpair(ptCloudObj, ptCloud_obj);
%         xlim([xlimits(1) xlimits(2)])
%         ylim([ylimits(1) ylimits(2)])
%         zlim([zlimits(1) zlimits(2)])
    
        view(player, ptCloud_obj.Location, ptCloud_obj.Intensity);
%         pause(0.5);

        view(player2, ptCloudOut.Location, ptCloudOut.Intensity);

        lidarData{end + 1}= ptCloud_obj;   
    
        frame_num = frame_num + 1;
        
    end

end
 
str_lidarData = ['Range_George@Albany0730', '.mat'];
save(str_lidarData, 'lidarData');
