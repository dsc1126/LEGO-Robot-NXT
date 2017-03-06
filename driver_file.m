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

map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map

%% Test functions
a = robotUltrascan();
% for i=1:4
%     %move(1000);
%     turn(90);
% end
testPos = [10 10];
testTarget = [40 40];
angle = 0;
path = pathPlanning(testPos, testTarget, map)
for i=1:length(path)-1
    angle = pathMove(path(i), angle, path(i+1));
end


%% Particle Filter
% @input: map
% @output: estimatedLocation, estimatedAngle

%% Path Planning
% @input: position, target, map
% @output: pathArray, lost

%% Path Move
% @input: currentPosition, nextPosition, currentAngle



%% Clean before program exit
COM_CloseNXT(handle); 
end