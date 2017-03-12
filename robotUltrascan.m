function [scanValues] = robotUltrascan()
    % Initialize the sound sensor by setting the sound sensor mode and input port. 
    OpenUltrasonic(SENSOR_4);

    % init values
    power = -20;
    Ports = [MOTOR_A];  % motorports for left and right wheel
    nrScans = 30;
    scanValues = zeros(nrScans,1);

    for i=1:1%nrScans
        % Get the current sound sensor value in dB.
        scanValues(i) = GetUltrasonic(SENSOR_4);

        % create motor object with defined variables
        mScan                       = NXTMotor(Ports);
        mScan.Stop('off'); % initialise motors
        mScan.SpeedRegulation       = false; % not sure what this actually mean
        mScan.Power                 = power;
        mScan.ActionAtTachoLimit    = 'Brake';
        
        % where are we?
        mScan.ResetPosition();
        data                = mScan.ReadFromNXT();
        pos                 = data.Position;
%         mScan.TachoLimit    = (360/nrScans)+pos;
        mScan.TachoLimit    = 360
        
        % move
        mScan.SendToNXT();
        mScan.WaitFor();
    end
    
    % move back
    mScan.Power                 = -power;
    mScan.TachoLimit            = 360;
    mScan.SendToNXT();
    mScan.WaitFor();

    % Close the sound sensor.
    CloseSensor(SENSOR_4);
end