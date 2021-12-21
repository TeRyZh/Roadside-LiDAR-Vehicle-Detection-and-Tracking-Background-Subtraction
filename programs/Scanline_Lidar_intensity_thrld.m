% veloReader = velodyneFileReader('D:\Dropbox\LiDAR_Data\VLP32c_0Degree162inch_FieldTest_2_ConstructionSite.pcap','VLP32C');
veloReader = velodyneFileReader('2021-10-13-12-56_Velodyne-VLS-128-Bakers_Basin_Data.pcap','VLS128');
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
numr_range = 300;
numc_range = 1000;

tensor_size = [total_channel,total_grid,total_frame];
One_SnapShot = NaN(1800,1);
STMAP_Data = zeros(tensor_size);
intensity_matrix = zeros(tensor_size);
Azimuth_matrix = zeros(tensor_size);
elevation_matrix = zeros(tensor_size);
frame_num = 1;

x_roi = [-200 200]; % left-right
y_roi = [-200 200];  % forward
z_roi = [-5 5]; %  height

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

    frame_x = [];
    frame_y = [];
    frame_z = [];
    
    for channel = 1: total_channel

        channel_LiDAR = squeeze(ptCloudObj.Location(channel,:,:));
        intensity_values = ptCloudObj.Intensity;
        
        x = channel_LiDAR(:,1)';
        y = channel_LiDAR(:,2)';
        z = channel_LiDAR(:,3)';
        
        frame_x = [frame_x x];
        frame_y = [frame_y y];
        frame_z = [frame_z z];
        
        % set up region of interest for object
        
        index = ((x > x_roi(1)) & (x < x_roi(2))) & (y > y_roi(1) & y < y_roi(2)) & (z > z_roi(1) & z < z_roi(2));
        
        x(~index) = NaN;
        y(~index) = NaN;
        z(~index) = NaN;
        intensity_values(channel, ~index) = 0;
        
        [azimuth,elevation,r] = cart2sph(x,y,z);
        
        degree = rad2deg(azimuth);
        degree(degree<0) = degree(degree<0) + 360;
        grids_idx = mod(floor(degree/azimuth_unit) + 1, total_grid);
        grids_idx(isnan(grids_idx))=0;
        
        for j = 1: length(grids_idx)

            if grids_idx(j) ~= 0

               if STMAP_Data(channel,grids_idx(j),frame_num) ~= 0

                   if STMAP_Data(channel,grids_idx(j),frame_num) > r(j)

                       STMAP_Data(channel,grids_idx(j),frame_num) =  r(j);
                       intensity_matrix(channel,grids_idx(j),frame_num) = intensity_values(channel, j);
                       Azimuth_matrix(channel,grids_idx(j),frame_num) = azimuth(j);
                       elevation_matrix(channel,grids_idx(j),frame_num) = elevation(j);

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
    
    range_img = pointcloud2image(frame_x', frame_y', frame_z', numr_range, numc_range);
%     figure
%     imshow(range_img);
    
    frame_num = frame_num + 1;

    if (frame_num > total_frame)
        break;
    end
    
end

%% Save Heat Map
% for ch = 1: total_channel
%     
%     heatMap_name = sprintf('heatMaps/heatMap_scanline_%d',ch);
%     single_heatmap =  imgaussfilt(squeeze(intensity_matrix(ch,:,:))); 
%     HeatMap(single_heatmap, 'Colormap', 'winter', 'Colorbar', true);
% %     hFig = gcf;
% %     saveas(hFig,heatMap_name,'png'); % You can use other image types as well and also `'fig'
%     
% end

%% DMD method for foreground detection
dmd_thrld = 0.1*255;
channel = 1;
single_heatmap =  squeeze(intensity_matrix(channel,:,:));
channel_foreground = single_heatmap;
single_heatmap = single_heatmap;
dmd_background = @(block_struct) background_dmd(block_struct.data);
block_size = [total_grid 1000];
channel_background = blockproc(single_heatmap,block_size,dmd_background);
channel_background_diff = abs(single_heatmap - channel_background);

channel_foreground(channel_background_diff <= dmd_thrld) = 0;
foreground_mask = channel_foreground;
foreground_mask(foreground_mask > 0) = 1;
foreground_mask=bwareaopen(foreground_mask,80);

figure
imshow(single_heatmap);

figure
imshow(channel_background);

figure
imshow(foreground_mask);

%% thresholding 
foreground = rowByRowThrld(single_heatmap);

figure
imshow(single_heatmap/255);

figure
imshow(foreground/255);

%% tensor decomposition

total_channel = 128;
total_grid = 1800;
total_frame = 1000;

num_components = 1;

x=1:total_grid; y=1:total_channel; t = 1 : total_frame;

figure(2)
model=parafac(intensity_matrix,num_components);
[A1,A2,A3]=fac2let(model);

subplot(3,1,1), plot(y,A1,'Linewidth',[2])
subplot(3,1,2), plot(x,A2,'Linewidth',[2])
subplot(3,1,3), plot(t,A3,'Linewidth',[2])

subplot(3,1,1), set(gca,'Xlim',[0 total_channel], 'Fontsize',[15])
subplot(3,1,2), set(gca,'Xlim',[0 total_grid], 'Fontsize',[15])
subplot(3,1,3), set(gca,'Xlim',[0 total_frame], 'Fontsize',[15])

% reconstruct Tensor
for i = 1: num_components
    
    a1 = squeeze(A1(:,i)); a2 = squeeze(A2(:,i)); a3 = squeeze(A3(:,i));
    
    if i == 1
        
        rec_tensor = a1.*(a2.') .* (reshape(a3, 1, 1, []));
        
    else
        
        rec_tensor = rec_tensor + a1.*(a2.') .* (reshape(a3, 1, 1, []));
        
    end
    
end

%% tensor and intensity matrix
channel = 30;

figure; imshow(squeeze(rec_tensor(channel,:,:))/255);
% figure; imshow(squeeze(intensity_matrix(channel,:,:))/255);

%%
% compute background model using DMD for each Channel
dmd_thrld = 0.1*255;
dmd_background = @(block_struct) background_dmd(block_struct.data);
block_size = [total_grid 1000];

sparse_fgs = {};
low_rank_bgs = {};
    
for channel = 1:total_channel

    channel_intensity = squeeze(intensity_matrix(channel,:,:));
    channel_foreground = channel_intensity;
    
    if sum(sum(~channel_foreground == 0)) ~= 0
        
        channel_background = blockproc(channel_intensity, block_size, dmd_background);
        channel_background_diff = abs(channel_intensity - channel_background);
        channel_foreground(channel_background_diff <= dmd_thrld) = 0;
        
    end
    
    foreground_mask = channel_foreground;
    foreground_mask(foreground_mask > 0) = 1;
%     foreground_mask=bwareaopen(foreground_mask,20);
    
%     figure; imshow(foreground_mask);
%     figure; imshow(channel_intensity);
%     HeatMap(sparse_fg, 'Colormap', 'jet');
%     HeatMap(low_rank_bg, 'Colormap', 'jet');
    
    sparse_fgs{end + 1} = foreground_mask;
    
    low_rank_bgs{end + 1} = 1- foreground_mask;

end
    
str_e = sprintf('DMD_resultS');

save(str_e, "sparse_fgs", "low_rank_bgs");

%% Filter out Background Points

filtered_range_data = zeros(total_channel,total_grid, total_frame);
filtered_azimuth = zeros(total_channel,total_grid, total_frame);
filtered_elevation = zeros(total_channel,total_grid, total_frame);
filtered_intensity = zeros(total_channel,total_grid, total_frame);

for channel = 1: total_channel
    
    range_data = squeeze(STMAP_Data(channel,:,1:total_frame));
    
    intensity_data = squeeze(intensity_matrix(channel,:,1:total_frame));
    azimuth_data = squeeze(Azimuth_matrix(channel,:,1:total_frame));
    elevation_data = squeeze(elevation_matrix(channel,:,1:total_frame));
    
%     index_bg_cell = index_bgs(channel);
%     bg_index = index_bg_cell{1,1};
    
    sparse_fg = sparse_fgs{channel};
    low_rank_bg = low_rank_bgs{channel};
    
    range_data((sparse_fg <= 0) == 1) = NaN;
    azimuth_data((sparse_fg <= 0) == 1) = NaN;
    elevation_data((sparse_fg <= 0) == 1) = NaN;
    intensity_data((sparse_fg <= 0) == 1) = 0;
    
%     filtered_channel_range_data = range_data;
%     filtered_channel_azimuth_data = azimuth_data;
%     filtered_channel_elevation_data = elevation_data;
%     filtered_channel_intensity_data = intensity_data;
    
    filtered_range_data(channel,:,:) = range_data;     % 32*1800*1000
    filtered_intensity(channel,:,:) = intensity_data;  % 32*1800*1000
    filtered_azimuth(channel,:,:) = azimuth_data;      % 32*1800*1000
    filtered_elevation(channel,:,:) = elevation_data;  % 32*1800*1000
end

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


