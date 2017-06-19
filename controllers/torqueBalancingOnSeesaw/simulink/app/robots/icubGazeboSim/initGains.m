ROBOT_DOF            = 23;
CONFIG.ON_GAZEBO     = true;

PORTS.IMU    = '/icubSim/inertial';
PORTS.NECK   = '/icubSim/head/state:o';

WBT_wbiList  = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';

sat.torque   = 34;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);

addpath(genpath('../matlab'));

model.robot.dofs = ROBOT_DOF;
    
seesaw           = struct;
% Height of the seesaw
%  ____________
%   *        *   | h
%     *    *     |
%       **       |
seesaw.h         = 0.1;

% Radius of the seesaw
seesaw.rho       = 0.362;

% Distance beteewn the center of rotation and the center of mass
seesaw.top       = 0.002; % seesaw.delta - (seesaw.rho - seesaw.h) ;
seesaw.delta     = seesaw.rho - seesaw.h + seesaw.top;
seesaw.inertia   = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
seesaw.mass      = 4.2;

seesaw.kind      = seesawKind;

% Distance of the foot from the seesaw center
seesaw.lFootDistanceCenter      =  0.1;
seesaw.rFootDistanceCenter      = -0.1;
seesaw.offset                   = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

switch seesaw.kind
    case 1 %Spherical seesaw
        seesaw.iota      = seesaw.mass/(seesaw.inertia);
        seesaw.invIota   = inv(seesaw.iota);
    case 2 %Semi cylidrical seesaw
        seesaw.iota      = [1;0;0]*transpose([1;0;0])*seesaw.mass/(seesaw.inertia);
        seesaw.invIota   = 0;
end

reg             = struct;
reg.pinvTol     = 1e-7;
reg.pinvDamp    = 1e-2;
reg.pinvDampA   = 1e-7;
reg.HessianQP   = 1e-5;
reg.impedances  = 0.1;
reg.dampings    = 0;
reg.pinvDampVb  = 1e-2;
model.seesaw    = seesaw;

% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01

%%Assumptions:
% 1) feet do not move with respect to the seesaw
% 2) feet are centered with the center of mass of the seesaw
% these assumptions are enforced (?) in the function as persistent
% variables
directionOfOscillation            = [0;1;0];
referenceParams                   = [0.00  0.1];  %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

noOscillationTime        = 0;   % If DEMO_LEFT_AND_RIGHT = 1, the variable noOscillationTime is the time, in seconds, 
                                % that the robot waits before starting the left-and-righ

%%Gains and references
gain.posturalProp  = diag([ 1   1   1 ...
                            1   1   1   1 ...
                            1   1   1   1 ...
                            1   1   1   1   1   1 ...
                            1   1   1   1   1   1]);
                        
gain.posturalDamp  = gain.posturalProp*0;
gain.P_SATURATION  = 0.30;

gain.PAngularMomentum  = 0 ;
gain.DAngularMomentum  = 1;

gain.PCOM              = diag([  1    1   1]);
gain.DCOM              = 2*sqrt(gain.PCOM)/50;
gain.ICOM              = diag([  0    0   0]);

gain.seesawKP          = 0.1;
gain.seesawKD          = 2*sqrt(gain.seesawKP);

gain.seesawKP_passive  = 0.1;
gain.seesawKD_passive  = 2*sqrt(gain.seesawKP);
gain.seesawKLambda     = 0.5;

% Friction cone parameters
numberOfPoints         = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                            % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1; % 1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot                             
gain.footSize                = [ -0.07 0.07  ;   % xMin, xMax
                                 -0.03 0.03 ];   % yMin, yMax    

fZmin                        = 20;

% Assumption: at time t=0, the seesaw is horizontal and the left foot of
% the robot is at a given distance from the contact point of the seesaw
% with the ground ( = world frame origin).

% Rotation between the world and the left foot (= seesaw orientation) at
% time 0 assuming the left foot rigidly attached to the seesaw
w_R_leftFoot0 = eye(3);

% P_l0 = initial position of left foot w.r.t. world frame
P_l0 = [0; seesaw.lFootDistanceCenter; (seesaw.h + seesaw.top)];

% transformation matrix
w_H_Lfoot0 = [w_R_leftFoot0,  P_l0;
              zeros(1,3),     1];

