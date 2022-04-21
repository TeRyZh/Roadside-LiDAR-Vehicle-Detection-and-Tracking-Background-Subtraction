function [ptCloud_roi_filtered] = roi_filter(ptCloudObj,roi_mask)

    % xLim and yLim are the real distance for mask
    % xLim is horizontal axis [-150,150]: columns
    % yLim is vertical axis [-150 150]: rows
    % zlimits is the vertical limits

    yLim = size(roi_mask,1);
    xLim = size(roi_mask,2);

    locations = ptCloudObj.Location;
    Intensity = ptCloudObj.Intensity;

    locations = reshape(locations,[],3);
    Intensity = reshape(Intensity,[],1);

    locations = locations(~isnan(locations(:,1)), :);
    Intensity = Intensity(~isnan(locations(:,1)), :);

    for i = 1 : size(locations, 1)
    
        idx_row = xLim/2 - round(locations(i,1));
            
        idx_col = yLim/2 - round(locations(i,2));

        if idx_col > xLim || idx_col < 1 || idx_row > xLim || idx_row < 1
            % not in the 2D X-Y plane
            locations(i,:) = NaN;
            Intensity(i,:) = 0;
%             
%         elseif isnan(locations(i, j, 2)) 
%             % non returnable data
%             locations(i,j,:) = NaN;
%             Intensity(i,j) = 0;
                
        elseif roi_mask(idx_row, idx_col) == 0
                
            locations(i,:) = NaN;
            Intensity(i,:) = 0;
                
        end
        
    end
    
    ptCloud_roi_filtered = pointCloud(locations, 'Intensity', Intensity);
      
end