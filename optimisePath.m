function [optimisedPath] = optimisePath(path) % remove points too close together
if(length(path)>2)
    threshold = 10;%100
    total_distance = sqrt((path(length(path),1)-path(1,1))^2+(path(length(path),1)-path(1,1))^2);
    if threshold <= total_distance % check if thresthod is less than total distance to target
        index = 1;
        optimisedPath = [];
        optimisedPath(index,1) = path(index,1); % add first by default
        optimisedPath(index,2) = path(index,2);
        index = index+1;
        curr = 1;
        for i=2:length(path)-1
            deltaX = path(i,1)-path(curr,1);
            deltaY = path(i,2)-path(curr,2);
            dist = sqrt(deltaX^2 + deltaY^2);
            if(dist > threshold)
                optimisedPath(index,1) = path(i,1);
                optimisedPath(index,2) = path(i,2);
                index = index+1;
                curr = i;
            end
        end
        deltaX = path(length(path),1)-path(length(path)-1,1);
        deltaY = path(length(path),2)-path(length(path)-1,2);
        dist = sqrt(deltaX^2 + deltaY^2);
        if(dist < threshold)
            optimisedPath(index-1,1) = path(length(path),1);
            optimisedPath(index-1,2) = path(length(path),2);
        else
            optimisedPath(index,1) = path(length(path),1);
            optimisedPath(index,2) = path(length(path),2);
        end
    else
        optimisedPath = path;
        disp('threshod out of range!!!');
    end
end
end
