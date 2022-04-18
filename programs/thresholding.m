function thrld_value=thresholding(distances)

%     INPUTS
%         graylevelIM :   gray level image
%     OUTPUT
%         ubBackInt   :   upperbound threshold value in the range [0 1];
%         lbBackInt   :   lowerbound threshold value in the range [0 1];

%     Triangle algorithm
%     This technique is due to Zack (Zack GW, Rogers WE, Latt SA (1977), 
%     "Automatic measurement of sister chromatid exchange frequency", 
%     J. Histochem. Cytochem. 25 (7): 74153, )
%     A line is constructed between the maximum of the histogram at 
%     (b) and the lowest (or highest depending on context) value (a) in the 
%     histogram. The distance L normal to the line and between the line and 
%     the histogram h[b] is computed for all values from a to b. The level
%     where the distance between the histogram and the line is maximal is the 
%     threshold value (level). This technique is particularly effective 
%     when the object pixels produce a weak peak in the histogram.

%     Use Triangle approach to compute threshold (level) based on a
%     1D histogram (lehisto). num_bins levels gray image. 

%     lehisto :   histogram of the gray level image
%     num_bins:   number of bins (e.g. gray levels)

%%  
%   Find maximum of histogram and its location along the x axis
    frame_size = length(distances);
    mini_diff = 0.3;  % minimum difference between backgrounds and foregrounds


    if nnz(distances) < frame_size * 0.2

        thrld_value = max(distances);

    else

        distances(distances==0) = [];
    
        %% coarse histogram assessment
    
       n_bins = round(max(distances));
    
       if n_bins == 0
           n_bins = 1;
       end
    
       [N, edges] = histcounts(distances, n_bins);
    
       [~, max_edges_num]=max(N);
       coarse_bin_interval = edges(2) - edges(1);
    
       if max_edges_num ~= length(edges) - 1  % maximum bin size has the highest counts
    
           dist_max = edges(max_edges_num) + coarse_bin_interval * 5;
    
           distances(distances > dist_max) = [];
    
       end
    
    %% fine edge bin size
    
        max_range = max(distances);

        fine_bin_num = 200;

        bin_half = round(max_range/fine_bin_num,2);
        
        step = bin_half*2;
        
        edges = -bin_half : step : (max_range + step);
        
        [N,edges] = histcounts(distances,edges);  %% edges are bin values, N is the count for each bin
    
        N=N+1; 
        lehisto=N;    
        [~,xmax]=max(N); % xmax is the index of histogram bins
        xmax=round(mean(xmax));   %can have more than a single value!
        
    %   Find location of first values.
        
        f_bin = 1;
        
    %   first and last points  pt0, pt1
        ptx0 = f_bin; pty0 = lehisto(f_bin);
        
        ptx1 = xmax-1; pty1 = lehisto(xmax);
        
        k = (pty0-pty1)/(ptx0-ptx1);
        
        % Parameters of line
        a = k; b = -1; c = pty1;     % line 1 ax+by+c=0
        
    %%  calculate  threshold value
    
    %   Compute distances
        if  xmax <= 2  % LIDAR hit on nearby infrastructure. ignore the cornor case
            thrld_value = 0;
    %         histogram(distances,edges)
        else
            x1=f_bin:xmax;
            y1=lehisto(x1)-1;
            L=[];
            for i=1:size(x1,2)
                l=(a*x1(i)+b*y1(i)+c)/sqrt(a.^2+b.^2);
                L=[L l];
            end
        
    %   Obtain threshold as the location of maximum L.    
        level=find(max(L)==L);
        thrld_value = edges(round(mean(level))) + bin_half;
    
        end
    end
