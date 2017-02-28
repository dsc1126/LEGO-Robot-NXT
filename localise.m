function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

%% Key Parameters
navigation = 0; %Set with value '1' to enable navigation, '0' to enable navigation  
num = 500;%800; %number of particles
Min_distance = 10; %minDistance from the walls
scans = 64; %number of scans in 360 degree
maxNumOfIterations = 100; %maximum number of loop
variance = 100; %variance
damp = 0.00000000001; %0.000000005; %damping factor
MotionNoise = 0.1;
TurningNoise = 0.001; %(pi/30);
%convergence_threshold = 2; %for both location and angle
transstd=2; % translation standard deviation in cm
orientstd=1.5; % orientation standard deviation in degrees
mutation_rate=0.01; %pencetage of respawn parcticles to randomrised locations
robot_size = 6;    
inner_boundary = map;%Inflation(map, robot_size);
Connecting_Distance=10; 

%% Create Matrices for initialization
particle_Scan = zeros(scans,num); 
difference = zeros(scans,num);
weight = zeros(num,1);
new_weight = zeros(scans,1);
newParticle = zeros(num, 3); % position(1,2), angle(3)
positions = zeros(num, 2);   
angles = zeros(num,1);
diff_mean= zeros(360,1);
first_move_done = 0;

botSim.setScanConfig(botSim.generateScanConfig(scans));

particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(Min_distance); %generate some random particles inside the map
    particles(i).setScanConfig(generateScanConfig(particles(i), scans));
    particles(i).setMotionNoise(MotionNoise);
    particles(i).setTurningNoise(TurningNoise);
end

%% Localisation code
n = 0;
stage = 0 %initialize, stage=1 if pos estimation ready, stage=2 if angle ready, stage=3 if target reached

while(n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
if stage == 0
    for i = 1:num
        if particles(i).insideMap() ==0
        particles(i).randomPose(0);
        end
        particle_Scan(:,i)= particles(i).ultraScan();
    
    %% Write code for scoring your particles    
        for j=1:scans
            Shifted_scan = circshift(particle_Scan(:,i),j); 
            difference(j,i) = sqrt(sum((Shifted_scan-botScan).^2)); 
            new_weight(j) = (1/sqrt(2*pi*variance))*exp(-((difference(j,i))^2/(2*variance))) + damp;
        end
            [max_weight, max_pos] = max(new_weight);
            weight(i) = max_weight; %choose weight on best pos among scans
            
            particle_angle = particles(i).getBotAng() + max_pos*2*pi/scans;
            particles(i).setBotAng(mod(particle_angle, 2*pi)); 
    end

    % Normalized all weights to have the sum of 1
    Normalized_weight = rdivide(weight,sum(weight)); %weights = weight./sum(weight);
  
    %% Write code for resampling your particles
    for i = 1:num
        old_i = find(rand() <= cumsum(Normalized_weight),1); % returns the first element satisfied cumsum of weights is just larger than random number between 0 and 1
        newParticle(i, 1:2) = particles(old_i).getBotPos();
        newParticle(i, 3) = particles(old_i).getBotAng();
    end
    
    for i = 1:num
        particles(i).setBotPos([newParticle(i,1), newParticle(i,2)]);
        particles(i).setBotAng(newParticle(i,3));
    end   
    
    % obtain particle positions and angles
     for i = 1:num
         positions(i,:) = particles(i).getBotPos();
         angles(i)=particles(i).getBotAng();
     end

    %Set the mean estimate
    Estimated_Bot = BotSim(modifiedMap);
    Estimated_Bot.setScanConfig(Estimated_Bot.generateScanConfig(scans));
    Estimated_Bot.setBotPos(mean(positions));
    Estimated_Bot.setBotAng(mean(angles));
    
%     Estimated_Bot_mean = BotSim(modifiedMap);
%     Estimated_Bot_mean.setScanConfig(Estimated_Bot_mean.generateScanConfig(scans));
%     Estimated_Bot_mean.setBotPos(mean(positions));
%     Estimated_Bot_mean.setBotAng(mean(angles));
%     
%     Estimated_Bot_mode = BotSim(modifiedMap);
%     Estimated_Bot_mode.setScanConfig(Estimated_Bot_mode.generateScanConfig(scans));
%     Estimated_Bot_mode.setBotPos(mode(positions));
%     Estimated_Bot_mode.setBotAng(mode(angles));
    
    if botSim.debug()
         figure(1)
         hold off; %the drawMap() function will clear the drawing when hold is off
         botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
         for i =1:num
             particles(i).drawBot(3); %draw particle with line length 3 and default color
         end
         Estimated_Bot.drawBot(50, 'r');
         drawnow;
     end 

    %% Write code to check for convergence          
    stdev_positions = std(positions);
    stdev_angles = std(angles);
    if stdev_positions < transstd %convergence_threshold
        if stdev_angles < orientstd %convergence_threshold
        disp('convergence done');

        stage = 1
        end
    end
end    % unblock/block together with next 'if stage == 0'

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
if stage == 0 %( stage == 0 || stage == 1 )
    for i=1:mutation_rate*num
        particles(randi(num)).randomPose(0);
    end 

    %% Write code to decide how to move next
    [max_distance, max_index] = max(botScan); 
    turn = (max_index-1)*2*pi/scans; %turn to the direction has maximum distance
    move = max_distance*0.7;
    
    if  first_move_done == 1   
        if rand()< 0.5 %improve robustness    
            if rand()< 0.1 %0.5 %turn left
                left_index=scans*1/4;
                turn = (left_index-1)*2*pi/scans;
                [left_distance] = botScan(left_index);
                move= left_distance*0.7;
            else %turn right
                right_index=scans*3/4;
                turn = (right_index-1)*2*pi/scans;
                [right_distance] = botScan(right_index);
                move= right_distance*0.7;
            end
        end    
    end
    first_move_done = 1;
%     if rand()<0.7 
%         [max_distance, max_index] = max(botScan); 
%         turn = (max_index-1)*2*pi/scans;
%         move = max_distance*0.8*rand();
%     else 
%         index=randi(scans); 
%         turn = (index-1)*2*pi/scans;
%         move= botScan(index)*0.8;
%     end

    botSim.turn(turn);        
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    
    for i =1:num %for all the particles.
          particles(i).turn(turn);
          particles(i).move(move);
    end
end
    
    %% Find best orientation
if stage == 1
botScan = botSim.ultraScan();

for i=1:360
    Estimated_BotScan = Estimated_Bot.ultraScan();
    diff_mean(i) = norm(Estimated_BotScan-botScan);
    Estimated_Bot.setBotAng(i*pi/180);
end

%find the best orientation for the mean estimate
[min_diff_mean, min_pos_mean] = min(diff_mean);
Estimated_Bot.setBotAng(min_pos_mean*pi/180); 

% botScan = botSim.ultraScan();
% diff_mean= zeros(360,1);
% diff_mode= zeros(360,1);
% for i=1:360    %Check scans of mode and mean estimates at every angle
%     Estimated_meanScan = Estimated_Bot_mean.ultraScan();
%     Estimated_modeScan = Estimated_Bot_mode.ultraScan();
%     diff_mean(i) = norm(Estimated_meanScan-botScan);
%     diff_mode(i) = norm(Estimated_modeScan-botScan);
%     Estimated_Bot_mean.setBotAng(i*pi/180);
%     Estimated_Bot_mode.setBotAng(i*pi/180);
% end
% 
% %find the best orientation for the mean estimate
% [min_diff_mean, min_pos_mean] = min(diff_mean);
% Estimated_Bot_mean.setBotAng(min_pos_mean*pi/180); 
% 
% %find the best orientation for the mode estimate
% [min_diff_mode, min_pos_mode]=min(diff_mode);
% Estimated_Bot_mode.setBotAng(min_pos_mode*pi/180);
% 
% if min_diff_mean < min_diff_mode %pick best from mean or mode estimates
%     Estimated_Bot = Estimated_Bot_mean;
% else
%     Estimated_Bot = Estimated_Bot_mode;
% end
  
Estimated_position = Estimated_Bot.getBotPos()
Estimated_angle = Estimated_Bot.getBotAng()
stage = 2
end

%% Path Planning

if stage == 2 
if navigation == 1

origin_x = min(map(:,1));
origin_y = min(map(:,1));
origin_x_diff = origin_x - 1;
origin_y_diff = origin_y - 1;

start_point = uint8(round(Estimated_position))
end_point = uint8(round(target))
start_x = start_point(:,1) + origin_x_diff;
start_y = start_point(:,2) + origin_y_diff;
end_x = end_point(:,1) + origin_x_diff;
end_y = end_point(:,2) + origin_y_diff;

map_x_min = uint8(round(min(inner_boundary(:,1),[],1))) + origin_x_diff;
map_x_max = uint8(round(max(inner_boundary(:,1),[],1))) + origin_x_diff;
map_y_min = uint8(round(min(inner_boundary(:,2),[],1))) + origin_y_diff;
map_y_max = uint8(round(max(inner_boundary(:,2),[],1))) + origin_y_diff;

map_x_size = map_x_max - map_x_min;
map_y_size = map_y_max - map_y_min;

grid_map = int8(zeros(map_y_size,map_x_size)); 
% target_cost_map = zeros(map_x_size,map_y_size); 
% start_cost_map = zeros(map_x_size,map_y_size);
% total_cost_map = zeros(map_x_size,map_y_size);


Num_lines = size(inner_boundary(:,1),1)-1;

for q = 1:(Num_lines-1)

    x_node = inner_boundary(q,1) - map_x_min - origin_x_diff;
    x_node_next = inner_boundary(q+1,1) - map_x_min - origin_x_diff;
    y_node = inner_boundary(q,2) - map_y_min - origin_x_diff;
    y_node_next = inner_boundary(q+1,2) - map_y_min - origin_x_diff;
    q = q+1;
    
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
    grid_map(y_low:y_high,x_low:x_high) = 1;
end

for q = Num_lines
    x_node = inner_boundary(q,1) - map_x_min - origin_x_diff;
    x_node_next = inner_boundary(1,1) - map_x_min - origin_x_diff;
    y_node = inner_boundary(q,2) - map_y_min - origin_x_diff;
    y_node_next = inner_boundary(1,2) - map_y_min - origin_x_diff;
    
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
    grid_map(y_node:y_node_next,x_node:x_node_next) = 1;
end    

GoalRegister=int8(zeros(map_y_size,map_x_size));
GoalRegister((end_y+origin_y_diff),(end_x+origin_x_diff))=1;






%% Used A-star Algorithm Function here
OptimalPath=ASTARPATH(start_x,start_y,grid_map,GoalRegister,Connecting_Distance)

if size(OptimalPath,2)>1
figure(9)
imagesc((grid_map))
    colormap(flipud(gray));

hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Path','Goal','Start')
%legend('Goal','Start','Path')

else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end

waypoints = OptimalPath - origin_x_diff

num_waypoints = numel(waypoints)/2;

target_waypoints_x = waypoints(1,2);
target_waypoints_y = waypoints(1,1);

u = 0;
for u = 2:(num_waypoints-1)
    waypoints_slope(u-2+1) = ((waypoints(u-2+1,1)-waypoints(u-2+2,1))/(waypoints(u-2+1,2)-waypoints(u-2+2,2)));
    u = u + 1;
end
for u = num_waypoints
    waypoints_slope(u-2+1) = ((waypoints(u-2+1,1)-waypoints(u-2+2,1))/(waypoints(u-2+1,2)-waypoints(u-2+2,2)));
end

%waypoints_slope = waypoints_slope_matrix(:,1) + waypoints_slope_matrix(:,end)

w = 0; %number of continued links
s = 2;
for s = 2:(num_waypoints-1)
    slope_diff = waypoints_slope(s) - waypoints_slope(s-1);
    if slope_diff > 0.3%*num_waypoints
        s = 1;
    else
        w = w + 1;
    end
end

target_waypoints_x = waypoints((num_waypoints-w),2) + origin_x_diff;
target_waypoints_y = waypoints((num_waypoints-w),1) + origin_x_diff;
start_point_x = double(start_x + origin_x_diff);
start_point_y = double(start_y + origin_y_diff);

for m = 1:(num_waypoints-1)  
angle = pi/180*atan2d(waypoints((num_waypoints-m),1)-waypoints((num_waypoints-m+1),1),waypoints((num_waypoints-m),2)-waypoints((num_waypoints-m+1),2))%+180)
distance = sqrt(((waypoints((num_waypoints-m),1)-waypoints((num_waypoints-m+1),1))^2)+(waypoints((num_waypoints-m),2)-waypoints((num_waypoints-m+1),2))^2)    

turn_angle = angle-Estimated_angle
Estimated_angle = Estimated_Bot.getBotAng()
robot_angle = rem(botSim.getBotAng(),6.28319)
botSim.turn(turn_angle);
robot_angle = rem(botSim.getBotAng(),6.28319)
Estimated_angle = robot_angle
%      for i =1:num %for all the particles.
%          angles(i)=particles(i).setBotAng(Estimated_angle);
%          particles(i).turn(turn_angle);
%      end
% Estimated_angle = Estimated_Bot.getBotAng()
botScan = botSim.ultraScan();  
if botScan(1)>= distance;
Estimated_position = Estimated_Bot.getBotPos()
robot_position = botSim.getBotPos()
botSim.move(distance);
robot_position = botSim.getBotPos()
%      for i =1:num %for all the particles.
%          positions(i,:) = particles(i).setBotPos(Estimated_position);
%          particles(i).move(distance);
%      end
% Estimated_position = Estimated_Bot.getBotPos()
else
    stage = 0
end


     
%      for i = 1:num
%          positions(i,:) = particles(i).setBotPos(Estimated_position);
%          angles(i)=particles(i).setBotAng(Estimated_angle);
%      end
%      
%      for i = 1:num
%          positions(i,:) = particles(i).getBotPos();
%          angles(i)=particles(i).getBotAng();
%      end
%      
%     stdev_positions = std(positions)
%     stdev_angles = std(angles)
     


    if botSim.debug()
         figure(1)
         hold off; %the drawMap() function will clear the drawing when hold is off
         botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
         botSim.drawBot(30,'g'); %draw robot with line length 30 and green
         for i =1:num
             particles(i).drawBot(3); %draw particle with line length 3 and default color
         end
         Estimated_Bot.drawBot(40, 'r');
         drawnow;
    end 
     
end

botScan = botSim.ultraScan();
Estimated_botScan = Estimated_Bot.ultraScan();    
break  
   
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
% angle = atan2d((target_waypoints_y-start_point_y),(target_waypoints_x-start_point_x)) + 180;
% distance = sqrt ( ((target_waypoints_y-start_point_y)^2) + ((target_waypoints_x-start_point_x)^2) );
% %      if distance > 10
% %          distance = distance / 2;
% %      end
% 
%    botSim.turn(pi-Estimated_angle+angle);
%        
%    botScan = botSim.ultraScan();  
% 
% if botScan(1)<= distance;
%     stage = 0;
% else
%     botSim.move(distance);
% end
% 
% botScan = botSim.ultraScan();
% Estimated_botScan = Estimated_Bot.ultraScan();
% 
% diff = (sum(Estimated_botScan-botScan)/scans);
% threshold = 3;
% % if (abs(diff) > threshold)
% %      stage = 0;
% % end
% 
% bot_location = Estimated_Bot.getBotPos();
%     
%     distance_to_target = sum((end_point(1)-bot_location(1)).^2)+(end_point(2)-bot_location(2).^2);
% if  distance_to_target < 9
%     stage = 3;
%     break
% end  
     
end
end

end
end
