function turn(angle)
    if(angle > 360)
           angle        = mod(abs(angle),360) % dont turn more than 360dgs
    elseif(angle < -360)
           angle        = -mod(abs(angle),360) % dont turn more than 360dgs
    end
    
    TurningSpeed        = 50;
    if(angle<0)
        TurningSpeed    = -TurningSpeed; % go in opposite direction if negative angle
        angle           = -angle;
    end
    
    if((angle/180)>1)
        angle = 360-angle; % turn in most efficient direction
        TurningSpeed    = -TurningSpeed; % go in opposite direction if negative angle
    end
    
    if(angle == 0) % do nothing
    else
        turnTicks           = abs(int16((291/45)*(angle/2)));      % assuming 45dgs turn is 290 ticks
        Ports               = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel

        mTurn1                      = NXTMotor(Ports(2)); % right motor
        mTurn1.SpeedRegulation      = false;  % we could use it if we wanted
        mTurn1.Power                = TurningSpeed;
        mTurn1.ActionAtTachoLimit   = 'Brake';

        % where are we?
        mTurn1.ResetPosition();
        data                        = mTurn1.ReadFromNXT();
        pos                         = data.Position;
        mTurn1.TachoLimit           = int16(turnTicks + pos);


        mTurn2          = mTurn1; % copy data
        mTurn2.Port     = Ports(1);   % left motor
        mTurn2.Power    = - mTurn1.Power; % reverse power

        mTurn1.SendToNXT(); % send both commands before wait
        mTurn2.SendToNXT();
        mTurn1.WaitFor();
        mTurn2.WaitFor();
    end
end

