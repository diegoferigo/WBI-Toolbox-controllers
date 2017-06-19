function desired_x_dx_ddx_CoM = trajectoryGenerator(xCoM0, t, referenceParams, CoM_y_delta, directionOfOscillation, noOscillationTime)

% Reference generator
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

% This is just for moving the CoM along y direction
% xcomDes(2) =  CoM_y_delta + xCoM0(2);

desired_x_dx_ddx_CoM = [xcomDes; xDcomDes; xDDcomDes];

end