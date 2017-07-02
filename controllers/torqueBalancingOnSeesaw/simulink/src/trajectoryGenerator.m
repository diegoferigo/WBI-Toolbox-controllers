function desired_x_dx_ddx_CoM = trajectoryGenerator(xCoM0, t, referenceParams, directionOfOscillation, noOscillationTime)

% Reference generator for robot CoM

% Amplitude
if t > noOscillationTime
    A = referenceParams(1);
else
    A = 0;
end

% Frequency
f          =  referenceParams(2);

xcomDes    =  xCoM0 + A*sin(2*pi*f*t)*directionOfOscillation;

xDcomDes   =  A*2*pi*f*cos(2*pi*f*t)*directionOfOscillation;

xDDcomDes  = -A*(2*pi*f)^2*sin(2*pi*f*t)*directionOfOscillation;

desired_x_dx_ddx_CoM = [xcomDes; xDcomDes; xDDcomDes];

end