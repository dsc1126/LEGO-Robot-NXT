function [botSim, Estimated_Bot] = PFL(botSim, modifiedMap,num, maxNumOfIterations,numscan,handle)
%Particle Filter Localisation Function
%numscan = 6;


botSim.setMap(modifiedMap);

%generate some random particles inside the map
particles(num,1) = BotSim; %how to set up a vector of objects


for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i), numscan));
   
    particles(i).setMotionNoise(0.1); %give the particles some motion noise
    particles(i).setTurningNoise(0.001);
    particles(i).setSensorNoise(1);
end

n = 0;
converged = 0;
variance = 100;   %variance
damp = 0;
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations   
    %botScan = botSim.ultraScan(); %get a scan from the real robot.
    botScan = robotUltrascan(numscan);

    
    %% Write code for updating your particles scans
    particle_Scan = cell(num,1);
    weight = zeros(num,1);
    
    for i=1:num
        if particles(i).insideMap() ==0
            particles(i).randomPose(0);
        end
        particle_Scan{i}= particles(i).ultraScan();
    end
  
    
    %% Write code for scoring your particles
    for i = 1:num
        for j=1:numscan
					Shifted_scan = circshift(particle_Scan{i},j);
					difference(j,i) = sqrt(sum((Shifted_scan-botScan).^2));
					new_weight(j) = (1/sqrt(2*pi*variance))*exp(-((difference(j,i))^2/(2*variance))) + damp;
				end

				[max_weight, max_pos] = max(new_weight);
				weight(i) = max_weight; %choose weight on best pos among scans

				particle_angle = particles(i).getBotAng() + max_pos*2*pi/numscan;
				particles(i).setBotAng(mod(particle_angle, 2*pi));

        end

    %Normalisation
		Normalized_weight = rdivide(weight,sum(weight));
		weight_sum = cumsum(Normalized_weight);


    %% Write code for resampling your particles
		for i = 1:num
			old_i = find(rand() <= weight_sum,1);
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
		Estimated_Bot.setScanConfig(Estimated_Bot.generateScanConfig(numscan));
		Estimated_Bot.setBotPos(mean(positions));
		Estimated_Bot.setBotAng(mean(angles));
        
        Estimated_Bot.drawBot(30,'g');

		%% Write code to check for convergence

		stdev_positions = std(positions);
		stdev_angles = std(angles);

		if stdev_positions < 1 %convergence_threshold
			if stdev_angles < 0.8 %convergence_threshold
				disp('convergence done');
				break;
			end
		end
%% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)
    

        for i=1:0.01*num
            particles(randi(num)).randomPose(0);
        end

    %% Write code to decide how to move next

	[max_botScan, max_index] = max(botScan);

    turn_angle = (max_index-1)*2*pi/numscan;
    turn(turn_angle*180/pi);
    
    collision_scan = collisionscan();%after turning the robot, scan nearby to check if there is collision may caused
    
    move = min(collision_scan)*0.4;
    
    
    
    
    %move = botScan(max_index)*0.2;
    moveRobot(move*10);    
    botSim.turn(turn_angle);        
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    
    %turn(turn_angle*180/pi);
    %moveRobot(move*10);
    
    
    for i =1:num %for all the particles.
          particles(i).turn(turn_angle);
          particles(i).move(move);
    end
end

%% checking

%botScan = botSim.ultraScan();
botScan = robotUltrascan(numscan);

for i=1:360
	Estimated_BotScan = Estimated_Bot.ultraScan();
	diff_mean(i) = norm(Estimated_BotScan-botScan);
	Estimated_Bot.setBotAng(i*pi/180);
end

[min_diff_mean, min_pos_mean] = min(diff_mean);
Estimated_Bot.setBotAng(min_pos_mean*pi/180);

NXT_PlayTone(1200,100, handle); %plays a tone
NXT_PlayTone(800,800, handle); %plays a tone


end

