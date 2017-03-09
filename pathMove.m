function [angle] = pathMove(position, angle, target)
    deltaX = target(1)-position(1);
    deltaY = target(2)-position(2);
    theta=atan(abs(deltaY)/abs(deltaX));
    degrees=180*theta/pi;
    if(deltaX>0 && deltaY>0)
        turn(-angle+degrees);
        angle=degrees;
    elseif(deltaX<0 && deltaY>0)
        turn(-angle+180-degrees);
        angle=pi-degrees;
    elseif(deltaX>0 && deltaY<0)
        turn(-angle+270+degrees);
        angle=270+degrees;
    elseif(deltaX<0 && deltaY<0)
        turn(-angle+180+degrees);
        angle=180+degrees;
    end
    move=round(sqrt((deltaX^2)+(deltaY^2)));
    if(move>0)
        moveRobot(move);
    end
end