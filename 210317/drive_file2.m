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

% Ports           = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel

% map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map=[0,0;66,0;66,44;44,44;44,66;110,66;110,110;0,110];  %default map

botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
start_position = [88 88];
start_angle = 0;
botSim.setBotPos(start_position);
botSim.setBotAng(start_angle);
% target = botSim.getRndPtInMap(10);  %gets random target.
target = [22 88];

botSim.drawMap();
drawnow;
tic %starts timer
%% Test functions
% ass = robotUltrascan(10);
% b=1;
% moveRobot(600);
% turn(180);
% for i=1:8
% %     moveRobot(200);
% %     turn(-180);
%     ass = robotUltrascan(10);
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

% [botSim, Estimated_Bot] = PFL(botSim, map,500,10, 20,handle);
% disp(Estimated_Bot.getBotPos())
% a = 1;
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

% Testing Johans version
% inflated_boundaries = boundary_inflation(map, 14); % alternative inflation function
% botSim2 = BotSim(inflated_boundaries,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
% path0 = pathPlanning2(botSim,map,target,start_position)*10

% Parameters for path planning only
modifiedMap = map;
scans = 30;
inner_boundary = map;
Connecting_Distance = 50;
botSim.setMap(modifiedMap);
botSim.setScanConfig(botSim.generateScanConfig(scans));

Estimated_Bot = BotSim(modifiedMap);
Estimated_Bot.setScanConfig(Estimated_Bot.generateScanConfig(scans));
Estimated_Bot.setBotPos(start_position);
Estimated_Bot.setBotAng(start_angle);

figure(1)
hold off; %the drawMap() function will clear the drawing when hold is off
botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
botSim.drawBot(30,'g'); %draw robot with line length 30 and green
Estimated_Bot.drawBot(50, 'r');
drawnow;

% start_position = Estimated_Bot.getBotPos();

waypoints = pathPlanning(start_position, target, map, Connecting_Distance);
optimisedPath = optimisePath(waypoints)
% for i=1:length(optimisedPath)
%     plot(optimisedPath(i,2),optimisedPath(i,1),'x')
% end
    
%% Path Move
% @input: currentPosition, nextPosition, currentAngle
 pathMoveError = pathMove(optimisedPath, Estimated_Bot, botSim);

% aries path plan with johans path move
% hold on
% mtx=zeros(size(waypoints));
% mtx(:,1)=waypoints(:,2)
% mtx(:,2)=waypoints(:,1)
% waypoints=mtx;
% plot(waypoints(1,1),waypoints(1,2),'bo')
% for i=1:length(waypoints)
%     plot(waypoints(i,1),waypoints(i,2),'x')
% end
% path=flipud(waypoints(:,1:2))*10;


%testing johans path move
% for i=1:length(path0)
%     plot(path0(i,1)/10,path0(i,2)/10,'x')
% end
% 
% figure
% path=optimisePath(path0);
% botSim.drawMap();
% drawnow;
% hold on
% for i=1:length(path)
%     plot(path(i,1)/10,path(i,2)/10,'x')
% end
% angle = 0;
% debug=1;      
% botSim.setBotPos([path(1,1)/10,path(1,2)/10])
% botSim.drawBot(10);
% for i=1:length(path)-1
%     angle = pathMove2([path(i,1),path(i,2)], angle, [path(i+1,1),path(i+1,2)],botSim,debug);
% end
%% Done!
NXT_PlayTone(1200,100, handle); %plays a tone
NXT_PlayTone(800,800, handle); %plays a tone
toc

%% Clean before program exit
COM_CloseNXT(handle); 
end
