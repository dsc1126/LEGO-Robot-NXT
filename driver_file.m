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
botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = botSim.getRndPtInMap(10);  %gets random target.

tic %starts timer

%% Test functions
% a = robotUltrascan();
% moveRobot(80);
% for i=1:4
%     moveRobot(300);
%     turn(90);
% end
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;
hold on;
testPos = [20 20];
testTarget = [20 80];
plot(testPos(1), testPos(2), 'gx')
plot(testTarget(1), testTarget(2), 'rx')
angle = 0;
path = pathPlanning(testPos, testTarget, map, botSim)
for i=1:length(path)-1
    angle = pathMove([path(i,1),path(i,2)], angle, [path(i+1,1),path(i+1,2)]);
end


%% Particle Filter
% @input: map
% @output: estimatedLocation, estimatedAngle

% [botSim, Estimated_Bot] = PFL(botSim, map,500, 30);
% disp(Estimated_Bot.getBotPos())

% returnedBot = localise(botSim,map,target); %Where the magic happens
% 
% resultsTime = toc %stops timer
% 
% %calculated how far away your robot is from the target.
% resultsDis =  distance(target, returnedBot.getBotPos())
%  
% robot_position = returnedBot.getBotPos()
% robot_angle = rem(returnedBot.getBotAng(),6.28319)

%% Path Planning
% @input: position, target, map
% @output: pathArray, lost



%% Path Move
% @input: currentPosition, nextPosition, currentAngle



%% Clean before program exit
COM_CloseNXT(handle); 
end