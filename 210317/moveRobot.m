function moveRobot(distance) % distance in milimeters
    % const will be determined by measurement
    const   = 42*pi/360; % diameter of wheel ~42mm, 360dgs~360click
    dist    = 1.0*distance/const;
    
    % motor speed
    power = 100;
    Ports = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel
    
     % create motor object with defined variables
    mStraight = NXTMotor(Ports); 
    mStraight.Stop('off'); % initialise motors
    mStraight.SpeedRegulation = false; % not sure what this actually mean
    mStraight.SmoothStart = true;
    mStraight.Power = power;
    mStraight.ActionAtTachoLimit = 'Brake';
    
    % reset position to 0
      mStraight.ResetPosition();
    
    % where do we want to go?
    % account for errors, i.e. if pos is not 0
    tacholimit              = round(dist);
    mStraight.TachoLimit    = tacholimit;
    
    % move
    mStraight.SendToNXT();
    mStraight.WaitFor();
    
    % where are we?
    data    = mStraight.ReadFromNXT();
    pos     = data.Position

    % check position after movement!
    mStraight.TachoLimit              = abs(round(pos-tacholimit))
    if mStraight.TachoLimit > 0
        if pos-tacholimit < 0
            mStraight.Power = -power;
        end
        % move
        mStraight.SendToNXT();
        mStraight.WaitFor();
    end

end