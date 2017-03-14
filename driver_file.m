function driver_file()

% verify that the RWTH - Mindstorms NXT toolbox is installed.
if verLessThan('RWTHMindstormsNXT', '4.01');
    error(strcat('This program requires the RWTH - Mindstorms NXT Toolbox ' ...
    ,'version 4.01 or greater. Go to http://www.mindstorms.rwth-aachen.de ' ...
    ,'and follow the installation instructions!'));
end%if

COM_CloseNXT all
close all
clear all
clc

handle = COM_OpenNXT(); %open usb port
COM_SetDefaultNXT(handle); % set default handle

Ports           = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel

map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
% botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
start_position = [90 80];
start_angle = pi;
botSim.setBotPos(start_position);
botSim.setBotAng(start_angle);
target = botSim.getRndPtInMap(10);  %gets random target.

botSim.drawMap();
drawnow;

%% Parameters for path planning only
    modifiedMap = map;
    scans = 30;
    inner_boundary = map;
    Connecting_Distance = 10;
    botSim.setMap(modifiedMap);
    botSim.setScanConfig(botSim.generateScanConfig(scans));

    Estimated_Bot = BotSim(modifiedMap);
    Estimated_Bot.setScanConfig(Estimated_Bot.generateScanConfig(scans));
    Estimated_Bot.setBotPos(start_position);
    Estimated_Bot.setBotAng(start_angle);
    
    figure(1)
    hold off; %the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
    %botSim.drawBot(30,'g'); %draw robot with line length 30 and green
    Estimated_Bot.drawBot(50, 'r');
    drawnow;

tic %starts timer
%% Test functions
% a = robotUltrascan(10);
% b=1;
% moveRobot(660);
% turn(90);
% for i=1:1
% %     moveRobot(100);
%     turn(90);
% end
% % botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
% botSim.drawMap();
% drawnow;
% hold on;
% testPos = [20 20];
% testTarget = [20 80];
% plot(testPos(1), testPos(2), 'gx')
% plot(testTarget(1), testTarget(2), 'rx')
% angle = 0;
% path = pathPlanning(testPos, testTarget, map, botSim)
% for i=1:length(path)-1
%     angle = pathMove([path(i,1),path(i,2)], angle, [path(i+1,1),path(i+1,2)]);
% end


%% Particle Filter
% @input: map
% @output: estimatedLocation, estimatedAngle

% [botSim, Estimated_Bot] = PFL(botSim, map,500, 30);
% disp(Estimated_Bot.getBotPos())
% 
% returnedBot = localise(botSim,map,target,handle); %Where the magic happens
% 
% resultsTime = toc %stops timer
% 
% %calculated how far away your robot is from the target.
% resultsDis =  distance(target, returnedBot.getBotPos())
%  
% robot_position = returnedBot.getBotPos()
% robot_angle = rem(returnedBot.getBotAng(),6.28319)

%% Path Planning
% @input: position, angle, target, map
% @output: pathArray, lost
waypoints = pathPlanning(start_position, target, map, Connecting_Distance);

%% Path Move
% @input: currentPosition, nextPosition, currentAngle
pathMoveError = pathMove(waypoints, Estimated_Bot, scans)

%% Clean before program exit
COM_CloseNXT(handle); 
end
