function [ptCloudOut,indices] = cropPointCloud(ptCloudIn,xLim,yLim,zLim)
    % This method selects the point cloud within limits and removes the
    % ego vehicle point cloud using findNeighborsInRadius
    locations = ptCloudIn.Location;
    locations = reshape(locations,[],3);
    insideX = locations(:,1) < xLim(2) & locations(:,1) > xLim(1);
    insideY = locations(:,2) < yLim(2) & locations(:,2) > yLim(1);
    insideZ = locations(:,3) < zLim(2) & locations(:,3) > zLim(1);
    validIndices = insideX & insideY & insideZ;
    
    indices = find(validIndices);
    ptCloudOut = select(ptCloudIn,indices);
end