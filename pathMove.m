function [pathMoveError] = pathMove(waypoints, Estimated_Bot, scans, botSim) %botSim argument is optional
pathMoveError = 0;    
num_waypoints = numel(waypoints)/2;

    for m = 1:(num_waypoints-1)  
        angle = pi/180*atan2d(waypoints((num_waypoints-m),1)-waypoints((num_waypoints-m+1),1),waypoints((num_waypoints-m),2)-waypoints((num_waypoints-m+1),2));
        distance = sqrt(((waypoints((num_waypoints-m),1)-waypoints((num_waypoints-m+1),1))^2)+(waypoints((num_waypoints-m),2)-waypoints((num_waypoints-m+1),2))^2)   

        Estimated_angle = Estimated_Bot.getBotAng()
        turn_angle = angle-Estimated_angle
        
        if nargin >= 4
            botSim.turn(turn_angle);
        end
        turn(int16(turn_angle*180/pi)); %turn the real robot
        Estimated_Bot.turn(turn_angle);
        Estimated_angle = Estimated_Bot.getBotAng();
        Estimated_BotScan = Estimated_Bot.ultraScan();
        %botScan = botSim.ultraScan();
         OpenUltrasonic(SENSOR_4);
         botScan = GetUltrasonic(SENSOR_4); %robotUltrascan(1);  %get a scan from ultrasonic sensor
         CloseSensor(SENSOR_4);
        
%          difference = sqrt(sum((Estimated_BotScan(1)-botScan(1)).^2))

%         if (botScan(1)>= distance*0.1)&&(difference < 2000);
            Estimated_position = Estimated_Bot.getBotPos();
            if nargin >= 4
                botSim.move(round(distance));
            end
            moveRobot(uint16(round(distance*10))); %move the real robot
            Estimated_Bot.move(round(distance));
            Estimated_position = Estimated_Bot.getBotPos();
%         else
%             pathMoveError = 1;
%             break
%         end

         figure(1)
         hold off; %the drawMap() function will clear the drawing when hold is off
         if nargin >= 4
             botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
             botSim.drawBot(30,'g'); %draw robot with line length 30 and green
         end

         Estimated_Bot.drawMap();
         Estimated_Bot.drawBot(40, 'r');
         drawnow;
    end
end
