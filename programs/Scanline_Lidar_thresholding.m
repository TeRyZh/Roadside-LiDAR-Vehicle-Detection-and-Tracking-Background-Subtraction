% veloReader = velodyneFileReader('E:\Dropbox\LiDAR_Data\VLP32c_0Degree162inch_FieldTest_2_ConstructionSite.pcap','VLP32C');
veloReader = velodyneFileReader('Alpha_Prime_NBStation_20200920_B.pcap','VLS128');
%%
xlimits = [-300 300];
ylimits = [-300 300];
zlimits = [-100 100];

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');
%%
total_channel = 128;
total_grid = 1800;
total_frame = 1000;
azimuth_unit = 360/total_grid;
tensor_size = [total_channel,total_grid,total_frame];
One_SnapShot = NaN(1800,1);
STMAP_Data = zeros(tensor_size);
intensity_matrix = zeros(tensor_size);
Azimuth_matrix = zeros(tensor_size);
elevation_matrix = zeros(tensor_size);
frame_num = 1;

new_pointcloud = {};
% new_pointcloud{end + 1} = dsf; 

while(hasFrame(veloReader) && player.isOpen())

    disp(veloReader.CurrentTime);
    ptCloudObj = readFrame(veloReader);

    
    view(player,ptCloudObj.Location,ptCloudObj.Intensity);
    pause(0.01);
    
%     if (frame_num < 20)
%         continue;
%     end
   
    for channel = 1: total_channel

        channel_LiDAR = squeeze(ptCloudObj.Location(channel,:,:));
        intensity_values = ptCloudObj.Intensity;
        
        x = channel_LiDAR(:,1)';
        y = channel_LiDAR(:,2)';
        z = channel_LiDAR(:,3)';
        [azimuth,elevation,r] = cart2sph(x,y,z);

        degree = rad2deg(azimuth);
        degree(degree<0) = degree(degree<0) + 360;
        grids_idx = mod(floor(degree/azimuth_unit) + 1, total_grid);
        grids_idx(isnan(grids_idx))=0;
        
        for j = 1: length(grids_idx)

            if grids_idx(j) ~= 0

               if STMAP_Data(channel,grids_idx(j),cnt) ~= 0
                   
                   if STMAP_Data(channel,grids_idx(j),cnt) > r(j)
                       
                       STMAP_Data(channel,grids_idx(j),cnt) =  r(j);
                       intensity_matrix(channel,grids_idx(j),cnt) = intensity_values(channel, j);
                       Azimuth_matrix(channel,grids_idx(j),cnt) = azimuth(j);
                       elevation_matrix(channel,grids_idx(j),cnt) = elevation(j);
                       
                   end
                   
               else

                   STMAP_Data(channel,grids_idx(j),frame_num) =  r(j);
                   Azimuth_matrix(channel,grids_idx(j),frame_num) = azimuth(j);
                   elevation_matrix(channel,grids_idx(j),frame_num) = elevation(j);
                   intensity_matrix(channel,grids_idx(j),frame_num) = intensity_values(channel, j);

               end

            end

        end

    end
    
    frame_num = frame_num + 1;

    if (frame_num > total_frame)
        break;
    end
    
end

%% Save Heat Map
% for ch = 1: channel
%     
%     heatMap_name = sprintf('heatMaps/heatMap_scanline_%d',ch);
%     single_heatmap =  squeeze(STMAP_Data(ch,:,:));
%     HeatMap(single_heatmap, 'Colormap', 'winter', 'Colorbar', true);
%     hFig = gcf;
%     saveas(hFig,heatMap_name,'png'); % You can use other image types as well and also `'fig'
%     
% end


%%
% compute background model using RPCA for each Channel

mask_data = zeros(total_channel, total_grid, total_frame);

for channel = 1: total_channel
    
    for each_azimuth = 1 : total_grid

        channel_azimuth_data = squeeze(STMAP_Data(channel,each_azimuth,:));

        non_zero_elements = nonzeros(channel_azimuth_data);
        
        if numel(non_zero_elements)/numel(channel_azimuth_data) > 0.4

            median_dist = median(non_zero_elements);

            if any(isoutlier(non_zero_elements))
                
%                 hist(non_zero_elements);
                
                index = isoutlier(non_zero_elements) & non_zero_elements < median_dist;
                
                if any(index)
                    
                    fg_pts = non_zero_elements(index);
                    
                    [sharedvals, fg_idx] = intersect(channel_azimuth_data, fg_pts,'stable');
                    channel_azimuth_data_mask = zeros(size(channel_azimuth_data));
                    channel_azimuth_data_mask(fg_idx) = 1;
                    sum(channel_azimuth_data_mask)
                    mask_data(channel, each_azimuth, :) = channel_azimuth_data_mask;
                    
                end
                
            end
            
        end
        

    end

end
    
%% Filter out Background Points

filtered_range_data = STMAP_Data;
filtered_azimuth = intensity_matrix;
filtered_elevation = Azimuth_matrix;
filtered_intensity = elevation_matrix;

filtered_range_data( mask_data == 0) = NaN;
filtered_azimuth( mask_data == 0) = NaN;
filtered_elevation( mask_data  == 0) = NaN;
filtered_intensity( mask_data == 0 ) = 0;

save("filtered_data", "filtered_azimuth", "filtered_elevation", "filtered_range_data", "filtered_intensity");
%%
% frame_num = 1000;
% total_channel = 32;
% total_grid = 1800;

[fg_x, fg_y, fg_z] = sph2cart(filtered_azimuth, filtered_elevation, filtered_range_data);

location = zeros(total_channel,total_grid, 3);

for frame_i = 1: total_frame
    
    location(:,:,1) = fg_x(:,:,frame_i); 
    location(:,:,2) = fg_y(:,:,frame_i);
    location(:,:,3) = fg_z(:,:,frame_i); 
    intensity = filtered_intensity(:, :, frame_i);
    
    view(player, location, intensity);
    pause(0.05);

end


