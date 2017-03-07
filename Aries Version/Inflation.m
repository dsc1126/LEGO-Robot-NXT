function inner_boundary = Inflation(map, robot_size)
% Function returns a map with inflated internal boundaries using robotSize

%% First we need to initialise empty arrays to create the inflated boundaries 
midpoint_x = zeros(1, size(map,1));
midpoint_y = zeros(1, size(map,1));
shifted_midpoint_x_a = zeros(1, size(map,1));
shifted_midpoint_y_a = zeros(1, size(map,1));
shifted_midpoint_x_b = zeros(1, size(map,1));
shifted_midpoint_y_b = zeros(1, size(map,1));
inside_point_x = zeros(1, size(map,1));
inside_point_y = zeros(1, size(map,1));
orientation = zeros(1, size(map,1));
inner_boundary = zeros(size(map,1), 2);

%% Next we need to extract vertices from adjacent map walls
for current_wall_index = 1:1:size(map,1)
    
    if current_wall_index < size(map,1)
        
        % we need to extract the 2 points where the ends of the walls meet 
        
        point_1 = [map(current_wall_index,1);map(current_wall_index,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);
        
        point_2 = [map(current_wall_index+1,1);map(current_wall_index+1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);
        
    elseif current_wall_index == size(map,1)
        
        
        
        point_1 = [map(current_wall_index,1);map(current_wall_index,2)];
        x_1 = point_1(1);
        y_1 = point_1(2);
        
        point_2 = [map(1,1);map(1,2)];
        x_2 = point_2(1);
        y_2 = point_2(2);
        
    end
    
    % Next we calculate the mindpoints 
    midpoint_x(current_wall_index) = (x_1 + x_2)/2;
    midpoint_y(current_wall_index) = (y_1 + y_2)/2;
    
    % Then we compute the current pose of the map wall
    orientation(current_wall_index) = atan( (y_2 - y_1)/(x_2 - x_1) );
    
    % Next we add the inflated midpoints of the boundaries to the current
    % walls of the map
    shifted_midpoint_x_a(current_wall_index) = midpoint_x(current_wall_index) + robot_size*cos(orientation(current_wall_index) + pi/2);
    shifted_midpoint_y_a(current_wall_index) = midpoint_y(current_wall_index) + robot_size*sin(orientation(current_wall_index) + pi/2);
    shifted_midpoint_x_b(current_wall_index) = midpoint_x(current_wall_index) + robot_size*cos(orientation(current_wall_index) - pi/2);
    shifted_midpoint_y_b(current_wall_index) = midpoint_y(current_wall_index) + robot_size*sin(orientation(current_wall_index) - pi/2);
    
    % Next we need to ensure that the inflated boundaries are not ontop or
    % outside the map walls
    [IN_a ON_a] = inpolygon(shifted_midpoint_x_a(current_wall_index), shifted_midpoint_y_a(current_wall_index), map(:,1),map(:,2));
    [IN_b ON_b] = inpolygon(shifted_midpoint_x_b(current_wall_index), shifted_midpoint_y_b(current_wall_index), map(:,1),map(:,2));
    
    
    
    if IN_a == 1 && ON_a == 0 && IN_b == 1 && ON_b == 0 % if the inflated midpoints are inside the walls  

        observer_state = [midpoint_x(current_wall_index); midpoint_y(current_wall_index)];
        current_target_node_a = [shifted_midpoint_x_a(current_wall_index);shifted_midpoint_y_a(current_wall_index)];
        current_target_node_b = [shifted_midpoint_x_b(current_wall_index);shifted_midpoint_y_b(current_wall_index)];
        
        % Next we check if the inflated midpoint is visible from the map
        % wall midpoint 
        visibility_a = line_of_sight(observer_state, current_target_node_a, map);
        visibility_b = line_of_sight(observer_state, current_target_node_b, map);
        
        % Now we discard any midpoints which dont have a direct line of
        % sight of the wall
        if visibility_a == 1
            
            inside_point_x(current_wall_index) = shifted_midpoint_x_a(current_wall_index);
            inside_point_y(current_wall_index) = shifted_midpoint_y_a(current_wall_index);
            
        elseif visibility_b == 1
            
            inside_point_x(current_wall_index) = shifted_midpoint_x_b(current_wall_index);
            inside_point_y(current_wall_index) = shifted_midpoint_y_b(current_wall_index);
            
        end
        
    elseif IN_a == 1 && ON_a == 0 % If inflated midpoint, a, is isnide, yet not ontop of the wall 
        
        inside_point_x(current_wall_index) = shifted_midpoint_x_a(current_wall_index);
        inside_point_y(current_wall_index) = shifted_midpoint_y_a(current_wall_index);
        
    elseif IN_b == 1 && ON_b == 0 
        
        inside_point_x(current_wall_index) = shifted_midpoint_x_b(current_wall_index);
        inside_point_y(current_wall_index) = shifted_midpoint_y_b(current_wall_index);
        
    end
    
end


%% Finally, we can calculate the vertices of he inflated boundaries
for current_wall_index = 1:1:size(map,1)
    
    if current_wall_index < size(map,1)
        
        x_a = inside_point_x(current_wall_index);
        y_a = inside_point_y(current_wall_index);
        theta_a = orientation(current_wall_index);
        
        x_b = inside_point_x(current_wall_index + 1);
        y_b = inside_point_y(current_wall_index + 1);
        theta_b = orientation(current_wall_index + 1);
        
    elseif current_wall_index == size(map,1)
        
        x_a = inside_point_x(current_wall_index);
        y_a = inside_point_y(current_wall_index);
        theta_a = orientation(current_wall_index);
        
        x_b = inside_point_x(1);
        y_b = inside_point_y(1);
        theta_b = orientation(1);
        
    end
    
    r_b_numerator = y_b - y_a - (x_b - x_a)*tan(theta_a);
    r_b_denominator = cos(theta_b)*tan(theta_a) - sin(theta_b);
    
    inner_boundary(current_wall_index,1) = x_b + r_b_numerator/r_b_denominator*cos(theta_b);
    inner_boundary(current_wall_index,2) = y_b + r_b_numerator/r_b_denominator*sin(theta_b);
    
end