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

%% Test function
a = robotUltrascan();
% for i=1:4
%     %move(1000);
%     turn(90);
% end
%% Particle Filter
% @input: map
% @output: estimatedLocation, estimatedAngle

%% Path Planning
% @input: position, angle, target, map
% @output: pathArray, lost

%% Path Move
% @input: currentPosition, nextPosition, map




%% Clean before program exit
COM_CloseNXT(handle); 
end