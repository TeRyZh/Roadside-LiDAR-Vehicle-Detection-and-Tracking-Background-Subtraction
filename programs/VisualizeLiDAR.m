% Visulization LiDAR Segmentation Results

addpath('D:\202111020LiDAR') 
lidar_file_name_1 = '2021-10-20-15-30-35_Velodyne-VLS-128-Data_Frame_0_to_8999.pcap'; 
load('2021-10-20-15-30-35_Velodyne-VLS-128-Data_Frame_0_to_8999_filtered.mat');

%%
xlimits = [-150 150];
ylimits = [-150 150];
zlimits = [-2 2];

player = pcplayer(xlimits,ylimits,zlimits);

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');

%%
veloReader = velodyneFileReader(lidar_file_name_1,'VLS128');

frame_num = 0;

while(hasFrame(veloReader) && player.isOpen())
    
    %     disp(veloReader.CurrentTime);
        ptCloudObj = readFrame(veloReader);  

        frame_num = frame_num + 1;

%         if frame_num == 7298

            pause(0.2);

            ptCloud_obj = lidarData{frame_num};

            ptCloud_obj = pcdenoise(ptCloud_obj); % remove noise
    
            pcshowpair(ptCloudObj, ptCloud_obj);
            xlim([xlimits(1) xlimits(2)])
            ylim([ylimits(1) ylimits(2)])
            zlim([zlimits(1) zlimits(2)])

%         end

        view(player, ptCloudObj.Location, ptCloudObj.Intensity);
        pause(0.1);


end

