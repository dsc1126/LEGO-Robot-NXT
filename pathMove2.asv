function [angle] = pathMove2(position, angle, target, botSim, debug)
    deltaX = target(1)-position(1);
    deltaY = target(2)-position(2);
    theta=atan(abs(deltaY)/abs(deltaX));
    degrees=180*theta/pi;
    if(deltaX>0 && deltaY>0)
        turn(-angle+degrees);
        if(debug==1)
            botSim.turn((-angle+degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=degrees;
    elseif(deltaX<0 && deltaY>0)
        turn(-angle+180-degrees);
        if(debug==1)
            botSim.turn((-angle+180-degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=180-degrees;
    elseif(deltaX>0 && deltaY<0)
        turn(-angle-degrees);
        if(debug==1)
            botSim.turn((-angle-degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=-degrees;
    elseif(deltaX<0 && deltaY<0)
        theta=atan(abs(deltaX)/abs(deltaY));
        degrees=180*theta/pi;
        turn(-angle+270-degrees);
        if(debug==1)
            botSim.turn((-angle+270-degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=270-degrees;
        
    elseif(deltaX==0 && deltaY>0)
        degrees=90;
        turn(-angle+degrees);
        if(debug==1)
            botSim.turn((-angle+degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=degrees;
    elseif(deltaX==0 && deltaY<0)
        degrees=270;
        turn(-angle+degrees);
        if(debug==1)
            botSim.turn((-angle+degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=degrees;
    elseif(deltaX>0 && deltaY==0)
        degrees=0;
        turn(-angle+degrees);
        if(debug==1)
            botSim.turn((-angle+degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=degrees;
    elseif(deltaX<0 && deltaY==0)
        degrees=180;
        turn(-angle+degrees);
        if(debug==1)
            botSim.turn((-angle+degrees)*pi/180);
            botSim.drawBot(10,'r');
        end
        angle=degrees;
    end
    
    move=round(sqrt((deltaX^2)+(deltaY^2)));
    if(move>0)
        moveRobot(move);
        if(debug==1)
            botSim.move(move/10);
            botSim.drawBot(10);
        end
    end
end