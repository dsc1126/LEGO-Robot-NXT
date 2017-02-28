function [botSim] = localise(botSim,map,target)

lost=1;
count=0;
while(lost==1 && count<2)
    [botSim,position,angle,lost,modifiedMap] = particleFilter(botSim,map);
    count=count+1;
end

if(lost==1)
    position = max(modifiedMap)/2; %put location in middle of the map
    angle = 0; %set angle to 0
else
    [botSim] = pathPlanning(botSim,modifiedMap,target,position,angle);
end

end