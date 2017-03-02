function turn(angle)
    TurningSpeed    = 50;
    turnTicks       = uint16((300/45)*(angle/2));      % assuming 45dgs turn is 219 ticks
    Ports           = [MOTOR_B; MOTOR_C];  % motorports for left and right wheel
    
    mTurn1                      = NXTMotor(Ports(2)); % is it needed to swap ports?
    mTurn1.SpeedRegulation      = false;  % we could use it if we wanted
    mTurn1.Power                = TurningSpeed;
    mTurn1.TachoLimit           = turnTicks;
    mTurn1.ActionAtTachoLimit   = 'Brake';
    
    
    mTurn2          = mTurn1;
    mTurn2.Port     = Ports(1);   % ports swapped again...
    mTurn2.Power    = - mTurn1.Power;
    
    mTurn1.SendToNXT(); % first turn angle/2 by driving one wheel forward
    mTurn1.WaitFor();
    
    mTurn2.SendToNXT(); % then turn angle/2 by driving the othew wheel backward
    mTurn2.WaitFor();
end

