function [scanValues] = collisionscan()
    % Initialize the sound sensor by setting the sound sensor mode and input port. 
    OpenUltrasonic(SENSOR_4);

    % init values
    power = -40;
    Ports = [MOTOR_A];  % motorports
    nrScans = 3;
    scanValues = zeros(3,1);

    % create motor object with defined variables
    mScan                       = NXTMotor(Ports);
    mScan.ActionAtTachoLimit    = 'Brake';
    tachoLimit                  = 45;
    mScan.TachoLimit            = tachoLimit;
    %mScan.Power                 = power;
    
    % reset position to 0
    mScan.ResetPosition();
    
%     for i=1:nrScans
%         % Get the current sound sensor value in dB.
%         scanValues(i) = GetUltrasonic(SENSOR_4);
%         
%         mScan.Stop('off'); % initialise motors
%         
%         % move
%         mScan.SendToNXT();
%         mScan.WaitFor();
%     
%     end
    mScan.Power                 = power;
    mScan.Stop('off'); % initialise motors
    mScan.SendToNXT();
    mScan.WaitFor();

    scanValues(1) = GetUltrasonic(SENSOR_4);
    
    mScan.Power                 = -power;
    mScan.Stop('off'); % initialise motors
    mScan.SendToNXT();
    mScan.WaitFor();
    
   
    scanValues(2) = GetUltrasonic(SENSOR_4);
    
    mScan.Stop('off'); % initialise motors
    mScan.SendToNXT();
    mScan.WaitFor();
    
    scanValues(3) = GetUltrasonic(SENSOR_4);

    
    mScan.Power                 = power;
    
    % move back same amount
    data                        = mScan.ReadFromNXT();
    pos1                         = abs(data.Position);
    mScan.Power                 = power;
    mScan.TachoLimit            = pos1;
    
    % reset position to 0
    mScan.ResetPosition();
    
    mScan.SendToNXT();
    mScan.WaitFor();
    
    % where are we?
    data    = mScan.ReadFromNXT();
    pos2     = abs(data.Position);
    
    % check positi0on after movement!
    mScan.TachoLimit              = abs(round(pos1-pos2));
    if mScan.TachoLimit > 0
        if pos1-pos2 < 0
            mScan.Power = -power;
        end
        % move
        mScan.SendToNXT();
        mScan.WaitFor();
    end
    
    
    % Close the sound sensor.
    CloseSensor(SENSOR_4);
end