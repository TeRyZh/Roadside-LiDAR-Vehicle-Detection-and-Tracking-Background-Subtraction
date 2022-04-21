addpath('E:\GeorgeSt_Velodyne') 
% lidar_file_name = '2021-10-20-15-30-35_Velodyne-VLS-128-Data_Frame_0_to_8999';
veloReader = velodyneFileReader('E:\GeorgeSt_Velodyne\Training Set\DATA_20211215_133000.pcap','VLS128');
% veloReader = velodyneFileReader([lidar_file_name '.pcap'],'VLS128');
%%
xlimits = [-150 150];
ylimits = [-150 150];
zlimits = [-10 0];

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');
%%
total_channel = 128;
total_grid = 1800;
total_frame = 2500;
azimuth_unit = 360/total_grid;

tensor_size = [total_channel,total_grid,total_frame];
range_matrix = zeros(tensor_size);
intensity_matrix = zeros(tensor_size);
Azimuth_matrix = zeros(tensor_size);
elevation_matrix = zeros(tensor_size);
frame_num = 0;

raw_lidarData = {};
% x_roi = [-100 100]; % left-right
% y_roi = [-100 100];  % forward
% z_roi = [-5 5]; %  height

% new_pointcloud = {};
% new_pointcloud{end + 1} = dsf; 

xLim = xlimits(2) - xlimits(1);
yLim = ylimits(2) - ylimits(1);

raw_lidarData = {};

while(hasFrame(veloReader) && player.isOpen())

%     disp(veloReader.CurrentTime);
    ptCloudObj = readFrame(veloReader);
    
    view(player, ptCloudObj.Location, ptCloudObj.Intensity);
%     pause(0.01);

%     raw_lidarData{end+1} = ptCloudObj;
    
    frame_num = frame_num + 1;
    
    if (frame_num > total_frame)
        
        break;
        
    end
    
%     if (frame_num < 20)
%         continue;
%     end

    for channel = 1: total_channel

        channel_LiDAR = squeeze(ptCloudObj.Location(channel,:,:));
%         size(channel_LiDAR)
        intensity_values = ptCloudObj.Intensity;
        
        x = channel_LiDAR(:,1)';
        y = channel_LiDAR(:,2)';
        z = channel_LiDAR(:,3)';
        
        [azimuth,elevation,range] = cart2sph(x,y,z);
        
        degree = rad2deg(azimuth);
        degree(degree<0) = degree(degree<0) + 360;
        grids_idx = mod(floor(degree/azimuth_unit) + 1, total_grid);
        grids_idx(grids_idx == 0) = total_grid -1;  % deal with 1799 + 1 grid
        grids_idx(isnan(grids_idx))=0;
        
        for j = 1: length(grids_idx)

            if grids_idx(j) ~= 0

               if range_matrix(channel,grids_idx(j),frame_num) ~= 0

                   if range_matrix(channel,grids_idx(j),frame_num) > range(j)

                       range_matrix(channel,grids_idx(j),frame_num) =  range(j);

                   end

               else

                   range_matrix(channel,grids_idx(j),frame_num) =  range(j);

               end

            end

        end

    end
    
%     range_img = pointcloud2image(frame_x', frame_y', frame_z', numr_range, numc_range);
%     figure
%     imshow(range_img);
    
end

%%

dist_max = 150;

range_thrld_matrix = ones(total_channel, total_grid) * dist_max;

for i = 1 : total_channel
    
    for j = 1 : total_grid
    
        distances = reshape(range_matrix(i,j,:),[],1);
        
        thrld_value = thresholding(distances);
        
        range_thrld_matrix(i,j) = thrld_value;
        
    end 
    
end
