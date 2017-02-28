function [botSim,position,angle,lost,modifiedMap] = particleFilter(botSim,map)

%This function returns botSim, and accepts, botSim and a map
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
sensorNoise = 1;
motionNoise = 0.1;
turningNoise = 0.05;

%generate some random particles inside the map
num =600; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
    particles(i).setSensorNoise(sensorNoise);
    particles(i).setMotionNoise(motionNoise);
    particles(i).setTurningNoise(turningNoise);
end

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged=0; %The filter has not converged yet
numberScans = 6; %The number of scans
turns=0; %used in movement
%clf; axis equal; hold on; botSim.drawMap(); %clear draws

while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    %% Write code for updating your particles scans

    %% Write code for scoring your particles
    for i = 1:num %for all the particles.
        particles_scan = particles(i).ultraScan(); %particle scan
        %weights2(i) = dot(botScan,particles_scan)/(norm(botScan)*norm(particles_scan));
        for j = 1:numberScans %for each of the scan values
            d(j) = sqrt(sum((particles_scan - botScan).^2)); %euclidian distance
            particles_scan = circshift(particles_scan,-1); %repeat at every orientation
        end
        %a=sum(particles_scan);
        %b=sum(botScan);
        %weights3(i) = (1/sqrt(2*pi*sensorNoise^2)) * exp(-(a-b).^2/(2*sensorNoise)); %gaussian distribution for optimal particle orientation
        [min_d, min_d_ind] = min(d); %find minimum euclidean distance (ED) to select correct orientation
        turn = (min_d_ind-1)*(2*pi()/numberScans); %set particle turning distance
        %particles(i).turn(turn); %move particle to correct orientation
        %weights4(i) = (1/sqrt(2*pi*sensorNoise^2)) * exp(-(botScan(min_d_ind)-min_d).^2/(2*sensorNoise)); %gaussian distribution for optimal particle orientation
        weights(i) = 1/min_d; %use min ED of selected orientation to obtain weightings
    end 
    weights = weights/sum(weights); %normalize
    %% Write code for resampling your particles
    %clf; axis equal; hold on; botSim.drawMap(); %clear draws
    
    %Linear relationship in best weights
    [B,index] = sortrows(weights'); %sorts weights in ascending order
    count=0;
    n_dead=round(num/2); %by default resample 50% of the particles
    
    for i=1:num
        if(B(i)==0)
            count=count+1; %count all particles with weight 0
        end
    end
    if(count>n_dead)
        n_dead=count; %chose the highest of counted 0-weight particles, or 50% of total
    end
    n_remain = num-n_dead;
    
    a=n_dead/sum(weights(index(n_remain:num))); %resample function
    resample=zeros(num,1); %array containing number of resamples per index
 
    for i=n_remain:num %only use the top 50%
        resample(i)=round(a*B(i));
    end
    
    count=1;
    ind=0;
    for i=1:n_dead
        particles(index(i)).setBotPos(particles(index(num-ind)).getBotPos()); %update position
        particles(index(i)).setBotAng(particles(index(num-ind)).getBotAng()); %update weights
        weights(index(i))=weights(index(num-ind)); %update weights
        if(mod(count,resample(num-ind))==0)
            ind=ind+1;
            count=1;
        else
            count=count+1;
        end
        if(resample(num-ind)==0) %if run out of index, restart
            ind=0;
        end
    end
%    %Cumulative sum
%    clf; axis equal; hold on; botSim.drawMap(); %clear draws
%     for i = 1 : num
%         particles(i).setBotPos(particles(find(rand <= cumsum(weights),1)).getBotPos());
%         particles(i).setBotAng(particles(find(rand <= cumsum(weights),1)).getBotAng());
%         particles(i).drawBot(3);
%     end
%     a=1;
% 
%     
%     %Random sample
%     index = randi([1, num-1]);  %random number for initial starting point on wheel
%     beta = 0;
%     max_weight = max(weights);
%     for ii = 1 : num
%         beta = beta + rand(1)*2*max_weight; %Add a random number between 0 and max twice the max weight
%         while beta > weights(index) %Only resample certain particles
%             beta = beta - weights(index);
%             index = rem((index+1),num)+1; %Calculate the remainder of index/num
%             weights(ii) = weights(index);%Update weights
%             particles(ii).setBotPos(particles(index).getBotPos()); %Update particle position
%             particles(ii).setBotAng(particles(index).getBotAng());%Update particle angle
%         end
%     end
    
%     %Linear relationship in best weights
%     [B,index] = sortrows(weights'); %sorts weights in ascending order
%     count=0;
%     n_dead=round(num/2); %by default resample 50% of the particles
%     
%     for i=1:num
%         if(B(i)==0)
%             count=count+1; %count all particles with weight 0
%         end
%     end
%     if(count>n_dead)
%         n_dead=count; %chose the highest of counted 0-weight particles, or 50% of total
%     end
%     n_remain = num-n_dead;
%     
%     a=n_dead/sum(weights(index((4*num/5):num)));
%     resample=zeros(num,1);
%     
%     for i=(4*num/5):num
%         resample(i)=round(a*B(i));
%     end
%     
%     count=0;
%     ind=0;
%     for i=1:n_dead
%         if(mod(count,resample(num-ind))==0)
%             ind=ind+1;
%             count=0;
%         end
%         particles(index(i)).setBotPos(particles(index(num-ind)).getBotPos());
%         particles(index(i)).setBotAng(particles(index(num-ind)).getBotAng());
%         %particles(index(i)).drawBot(3);
%         count=count+1;
% 
%     end
%    %Cumulative sum
%    clf; axis equal; hold on; botSim.drawMap(); %clear draws
%     for i = 1 : num
%         particles(i).setBotPos(particles(find(rand <= cumsum(weights),1)).getBotPos());
%         particles(i).setBotAng(particles(find(rand <= cumsum(weights),1)).getBotAng());
%         particles(i).drawBot(3);
%     end
%     a=1;

    %Best 10 particles
    [B,index] = sortrows(weights'); %sorts weights in ascending order
    count=0;
    n_dead=round(num/2); %by default resample 50% of the particles
    
    for i=1:num
        if(B(i)==0)
            count=count+1; %count all particles with weight 0
        end
    end
    if(count>n_dead)
        n_dead=count; %chose the highest of counted 0-weight particles, or 50% of total
    end
    n_remain = num-n_dead;
    
    ind=0;
    for i=1:n_dead
        if(mod(i,num/10)==0)
            ind=ind+1;
        end
        particles(index(i)).setBotPos(particles(index(num-ind)).getBotPos()); %update position
        particles(index(i)).setBotAng(particles(index(num-ind)).getBotAng()); %update angle
        weights(index(i))=weights(index(num-ind));
    end
    
  
    
    %% Write code to check for convergence  
    xPos = zeros(num,1);
    yPos = zeros(num,1);
    ang = zeros(num,1);
    [B,index] = sortrows(weights'); %sorts weights in ascending order
    for i = 1:num
        temp = particles(index(i)).getBotPos();
        xPos(i,1) = temp(1);
        yPos(i,1) = temp(2);
        ang(i,1) = particles(i).getBotAng();
    end
    deviation = sqrt( mean( ((xPos)-mean(xPos)).^2 ) + mean( ((yPos)-mean(yPos)).^2 ));
    angDeviation = std(ang);
    stdNum=2;
    if(n>20)
        stdNum=5;
    end
    if(deviation<stdNum && angDeviation<stdNum)
        converged=1;
        position = [mean(xPos) mean(yPos)];
        part=round(length(B)*0.85);
        angle=mean(ang(part:num)); %mean of the top 15%
        lost=0;
    end
    
	
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    if(converged==0)
        [B,index] = sortrows(weights'); %sorts weights in ascending order
        percentage=(num/10); %10 percent of total amount of particles to be respawned
        for i = 1:round(percentage) %rounds percentage to closest integer
            particles(index(i)).randomPose(10); 
        end    
    end
    
    %% Write code to decide how to move next
    % move random direction
    % use 'inpolygon' to draw lines 
    if(converged==0)
        [A,index]=max(botScan);
        turn=(((2*pi)/numberScans)*(index-1));
        botSim.turn(turn); %move in the direction where is most space
        move=(A/4)+(rand*(3*A/4));
        botSim.move(move); %move randomly 1-20
        for i = 1:num %move particles similarly
            particles(i).turn(turn);
            particles(i).move(move);
        end
    end
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        for i =1:num
            particles(i).drawBot(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
  
end
if(converged==0) %if we didnt localise
    position = [0 0];
    angle = 0;
    lost=1;
end
end
