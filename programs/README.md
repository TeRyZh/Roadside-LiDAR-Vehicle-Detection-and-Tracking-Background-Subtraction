This folder contains functions developed for LiDAR background subtraction

#### Step 0. Before running the Algorithms, you need to prepare *beams_elevation.mat* and *George_st_ROI.png* for your own LiDAR data. 
- a. if you don't use Region-of-Interest (ROI), you need to comment out the following line of code. It doesn't impact the detection results. 
~~~
   [ptCloud_roi_filtered] = roi_filter(ptCloudOut,roi_mask); 
~~~
- b. *beams_elevation.mat* is Vertical Angles (ω) by Laser ID and Model, which can be found in the LiDAR Manual.

#### Step 1. Run *Triangle Thresholding.m* to obtain the *range_thrld_matrix.mat* that stored the threshold value for each elevation-azimuth grid

#### Step 2. Run *Background Filter.m* to filter out background and only preserve the foreground points. 

#### Step 3. Run *detectGIF.m* to generate bounding box detection. 



## Data Sample from Velody Prime 

[LiDAR Data Download Link](https://drive.google.com/file/d/167fXezNrgCpFmZod3yZwJsxRtyh0HfOx/view?usp=sharing)


[Albany St & George St, New Brunswick](https://www.google.com/maps/place/Albany+St+%26+George+St,+New+Brunswick,+NJ+08901/data=!4m2!3m1!1s0x89c3c6517121901d:0xdde5d4f0994007a?sa=X&ved=2ahUKEwiw0ouEkaT3AhVK3KQKHdlzBuIQ8gF6BAgCEAE)
