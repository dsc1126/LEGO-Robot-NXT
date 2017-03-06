function [path] = pathPlanning(position, target, map)
        limsMin = min(map); % minimum limits of the map
        limsMax = max(map); % maximum limits of the map
        if(limsMin(1)<0) %compensate for negative map start
            target(1)=target(1)-limsMin(1);
            position(1)=position(1)-limsMin(1);
        end  
        if(limsMin(2)<0)
            target(2)=target(2)-limsMin(2);
            position(2)=position(2)-limsMin(2);
        end  
        dims = limsMax-limsMin; %dimension of the map
        res = 0.5; %sampling resouloution in cm 
        iterators = dims/res;
        iterators = ceil(iterators); %to counteract 1 based indexing
        mapArray = zeros(iterators); %preallocate for speed
        hold on    
        %loops through the grid indexes and tests if they are inside the map
        for i = 1:iterators(2)
            for j = 1:iterators(1)
                testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing
                %notice, that i and j have also been swapped here so that the only
                %thing left to do is invert the y axis.
                mapArray(i,j) = botSim.pointInsideMap(testPos);
            end
        end
        mapArray=flipud(mapArray); %inverts y axis
        a=size(mapArray);
        for i=1:a(1) %binary flip 1 to 0 etc
            for j=1:a(2)
                if(mapArray(i,j)==0)
                    mapArray(i,j)=1;
                else
                    mapArray(i,j)=0;
                end
            end
        end   
        tmpMap = robotics.BinaryOccupancyGrid(mapArray, 2); %get a Occupancy Grid for path planning
        robotRadius = 1; %simulated value, not to go too close to edge
        mapInflated = copy(tmpMap);
        inflate(mapInflated,robotRadius); %inflates boundaries to avoid collission 
        prm = robotics.PRM; %define path planner
        prm.Map = mapInflated;
        prm.NumNodes = 300; %nr of nodes for path planning
        if(limsMax(1)>200 || limsMax(2)>200)
            prm.NumNodes = 500; %if the map is big
        end
        prm.ConnectionDistance = 25; %max distance between nodes
        startLocation = position;
        endLocation = target;
        path = findpath(prm, startLocation, endLocation); %path planning
        while isempty(path)
            prm.NumNodes = prm.NumNodes + 10; % No feasible path found yet, increase the number of nodes
            
            update(prm); %Use the |update| function to re-create the PRM roadmap with the changed attribute
            
            path = findpath(prm, startLocation, endLocation); % Search for a feasible path with the updated PRM
        end
        if(limsMin(1)<0) %compensate back for negative map start
            for i = 1:length(path)
                path(i,1)=path(i,1)+limsMin(1);
            end
            target(1)=target(1)+limsMin(1);
        end
        if(limsMin(2)<0)
            for i = 1:length(path)
                path(i,2)=path(i,2)+limsMin(2);
            end
            target(1)=target(1)+limsMin(1);
        end
end


