function [waypoints] = pathPlanning(position, target, map, Connecting_Distance)

origin_x = min(map(:,1));
origin_y = min(map(:,2));
origin_x_diff = 1 - origin_x;
origin_y_diff = 1 - origin_y;

start_point = round(position);
end_point = round(target);
start_x = uint16(start_point(:,1) + origin_x_diff);
start_y = uint16(start_point(:,2) + origin_y_diff);
end_x = uint16(end_point(:,1) + origin_x_diff);
end_y = uint16(end_point(:,2) + origin_y_diff);

map_x_min = uint16(round(min(map(:,1),[],1)) + origin_x_diff);
map_x_max = uint16(round(max(map(:,1),[],1)) + origin_x_diff);
map_y_min = uint16(round(min(map(:,2),[],1)) + origin_y_diff);
map_y_max = uint16(round(max(map(:,2),[],1)) + origin_y_diff);

map_x_size = map_x_max - map_x_min +10; %+ 1;
map_y_size = map_y_max - map_y_min +10; %+ 1;

grid_map = int8(zeros(map_y_size,map_x_size)); 
inflated_grid_map = int8(zeros(map_y_size,map_x_size)); 

Num_lines = size(map(:,1),1)-1;

for q = 1:Num_lines    
    x_node = map(q,1) - map_x_min + origin_x_diff;
    x_node_next = map(q+1,1) - map_x_min + origin_x_diff;
    y_node = map(q,2) - map_y_min + origin_y_diff;
    y_node_next = map(q+1,2) - map_y_min + origin_y_diff;  
    
    if x_node > x_node_next
        x_high = x_node;
        x_low = x_node_next;
    else
        x_high = x_node_next;
        x_low = x_node;
    end   
    
    if y_node > y_node_next
        y_high = y_node;
        y_low = y_node_next;
    else
        y_high = y_node_next;
        y_low = y_node;
    end
    
    if (y_high == y_low || x_high == x_low)
        grid_map(y_low:y_high,x_low:x_high) = 1;
    elseif (y_node_next>y_node && x_node_next>x_node)% || (y_node_next<y_node && x_node_next<x_node)
        for i = 1:(x_high-x_low)    
        grid_map(round(y_node+(y_node_next-y_node)*i/(x_node_next-x_node)),(x_node+i)) = 1;
        end
    elseif (y_node_next<y_node && x_node_next<x_node)
        for i = 1:(x_high-x_low)    
        grid_map(round(y_node-(y_node-y_node_next)*i/(x_node-x_node_next)),(x_node-i)) = 1;
        end  
    elseif (y_node_next>y_node && x_node_next<x_node)
        for i = 1:(x_high-x_low)
        grid_map(round(y_node+(y_node_next-y_node)*i/(x_node_next-x_node)),x_node-i) = 1;
        end    
    elseif (y_node_next<y_node && x_node_next>x_node)
        for i = 1:(x_high-x_low)
        grid_map(round(y_node-(y_node-y_node_next)*i/(x_node_next-x_node)),x_node+i) = 1;
        end    
    end   
end

for m = 16:(map_x_size-15)
 	for n = 16:(map_y_size-15)
    	if grid_map(n,m)==1
            for p = 1:15
             inflated_grid_map(n-p,m-p)=1;
             inflated_grid_map(n-p,m+p)=1;
             inflated_grid_map(n+p,m-p)=1;
             inflated_grid_map(n+p,m+p)=1;
            end          
        end
    end
end
 
GoalRegister=int8(zeros(map_y_size,map_x_size));
GoalRegister((end_y),(end_x))=1;


%% A-star Algorithm Function for path planning

OptimalPath=ASTARPATH(start_x,start_y,inflated_grid_map,GoalRegister,Connecting_Distance);

if size(OptimalPath,2)>1
    figure(9)
    imagesc((inflated_grid_map))
    %imagesc((grid_map))
    colormap(flipud(gray));

    hold on
    plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
    plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
    plot(OptimalPath(:,2),OptimalPath(:,1),'r')
    legend('Path','Goal','Start') %legend('Goal','Start','Path')
else 
    pause(1);
    h=msgbox('No path exists to the Target!','warn');
    uiwait(h,5);
end

%waypoints = OptimalPath - origin_x_diff;
waypoints = [(OptimalPath(:,1)-origin_y_diff),(OptimalPath(:,2)-origin_x_diff)];

target_waypoints_x = waypoints(1,2);
target_waypoints_y = waypoints(1,1);

end
