function move(distance)
    % const will be determined by measurement
    const   = 1;
    dist    = distance*const;
    
    % motor speed
    power = -50;
    Ports = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel
    
     % create motor object with defined variables
    mStraight = NXTMotor(Ports); 
    mStraight.Stop('off'); % initialise motors
    mStraight.SpeedRegulation = false; % not sure what this actually mean
    mStraight.Power = power;
    mStraight.ActionAtTachoLimit = 'Brake';
    
    % where are we?
    mStraight.ResetPosition();
    data    = mStraight.ReadFromNXT();
    pos     = data.Position;
    
    % where do we want to go?
    % account for errors, i.e. if pos is not 0
    tacholimit              = dist + pos;
    mStraight.TachoLimit    = tacholimit;
    
    % move
    mStraight.SendToNXT();
    mStraight.WaitFor();
end