function [angle] = pathMove(position, angle, target)
    deltaX = target(1)-position(1);
    deltaY = target(2)-position(2);
    theta=atan(abs(deltaY)/abs(deltaX));
    if(deltaX>0 && deltaY>0)
        turn(-angle+theta);
        angle=theta;
    elseif(deltaX<0 && deltaY>0)
        turn(-angle+pi-theta);
        angle=pi-theta;
    elseif(deltaX>0 && deltaY<0)
        turn(-angle+(3*pi/2)+theta);
        angle=(3*pi/2)+theta;
    elseif(deltaX<0 && deltaY<0)
        turn(-angle+pi+theta);
        angle=pi+theta;
    end
    move=round(sqrt((deltaX^2)+(deltaY^2)));
    if(move>0)
        moveRobot(move);
    end
end