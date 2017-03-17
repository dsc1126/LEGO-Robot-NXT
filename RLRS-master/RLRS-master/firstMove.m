function [move_distance, turn_angle] = firstMove(botScanCopy,scans)
    moved = 0;
    while(moved == 0)
        [max_distance, max_index] = max(botScanCopy);
        if(max_index == length(botScanCopy))
            lowerIndex = max_index - 1;
            upperIndex = 1;
        elseif(max_index == 1)
            lowerIndex = length(botScanCopy);
            upperIndex = max_index + 1;
        else
            lowerIndex = max_index - 1;
            upperIndex = max_index + 1;
        end

        if((botScanCopy(lowerIndex)>= 0.75*max_distance) && (botScanCopy(upperIndex)>= 0.75*max_distance))
            turn_index = max_index;
            moved = 1;
            turn_angle = (turn_index-1)*2*pi/scans; %turn to the direction has maximum distance
            move_distance = max_distance*0.5;
        else
            botScanCopy(max_index) = uint16(max_distance*0.8);
        end
    end
end