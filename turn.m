function turn(angle)
    TurningSpeed    = 50;
    turnTicks       = uint16((300/45)*(angle/2));      % assuming 45dgs turn is 219 ticks
    Ports           = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel
    
    mTurn1                      = NXTMotor(Ports(2)); % is it needed to swap ports?
    mTurn1.SpeedRegulation      = false;  % we could use it if we wanted
    mTurn1.Power                = TurningSpeed;
    mTurn1.ActionAtTachoLimit   = 'Brake';
    
    % where are we?
    mTurn1.ResetPosition();
    data    = mTurn1.ReadFromNXT();
    pos     = data.Position;
    mTurn1.TachoLimit           = turnTicks + pos;
    
    
    mTurn2          = mTurn1;
    mTurn2.Port     = Ports(1);   % ports swapped again...
    mTurn2.Power    = - mTurn1.Power;
    
    mTurn1.SendToNXT(); % first turn angle/2 by driving one wheel forward
    mTurn1.WaitFor();
    
    % re-check where we are
    mTurn2.ResetPosition();
    data    = mTurn2.ReadFromNXT();
    pos     = data.Position;
    mTurn2.TachoLimit           = turnTicks + pos;
    
    mTurn2.SendToNXT(); % then turn angle/2 by driving the othew wheel backward
    mTurn2.WaitFor();
end

