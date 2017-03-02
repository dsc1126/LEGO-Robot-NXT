function [scanValues] = robotUltrascan()
    % Initialize the sound sensor by setting the sound sensor mode and input port. 
    OpenSound(SENSOR_1, 'DB');

    % init values
    power = -40;
    Ports = [MOTOR_A];  % motorports for left and right wheel
    nrScans = 20;
    scanValues = zeros(nrScans,1);

    for i=1:nrScans
        % Get the current sound sensor value in dB.
        scanValues(i) = GetSound(SENSOR_1)

        % create motor object with defined variables
        mScan                       = NXTMotor(Ports);
        mScan.Stop('off'); % initialise motors
        mScan.SpeedRegulation       = false; % not sure what this actually mean
        mScan.Power                 = power;
        mScan.ActionAtTachoLimit    = 'Brake';
        mScan.TachoLimit            = 360/nrScans;

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
    CloseSensor(SENSOR_2);
end