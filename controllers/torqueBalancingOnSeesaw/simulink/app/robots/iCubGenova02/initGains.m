ROBOT_DOF      = 23;
WBT_wbiList    = 'ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP';
sat.torque     = 15;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
model.robot.dofs       = ROBOT_DOF;

%% Seesaw parameters
seesaw           = struct;

% Height of the seesaw
%  ___________
% |           |  ---> vertical border
%  *         *   | 
%   *      *     | h
%     *  *       |
seesaw.h          = 0.11;
seesaw.metalPlate = 0.004;

% Add vertical borders
if CONFIG.SEESAW_WITH_VERTICAL_BORDER == 1
    seesaw.h     = seesaw.h + 0.046;
end

% Radius of the seesaw
seesaw.rho       = 0.362;
seesaw.CoM_z     = seesaw.h + seesaw.metalPlate*0.5;

% Seesaw inertia and mass (no IMU)
seesaw.inertia   = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
seesaw.mass      = 4.2;

% WITH IMU ON SEESAW
if CONFIG.SEESAW_HAS_IMU == 1
    % with the IMU, the CoM of the seesaw is about 2 centimeters below the
    % metal plate
    shiftCoM       = 0.02;
    seesaw.CoM_z   = seesaw.h - shiftCoM;
    seesaw.inertia = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);
    seesaw.mass    = 6.2;
end

% Distance between the center of rotation and the center of mass. If the
% seesaw has no imu, the center of mass is assumed to be on the surface of
% the seesaw. Otherwise, the CoM is a bit lower
seesaw.delta     = seesaw.rho - seesaw.CoM_z;

% Distance between the seesaw/feet contact plane and the seesaw CoM
seesaw.top       = (seesaw.h + seesaw.metalPlate) - seesaw.CoM_z;

% Shape of the seesaw
seesaw.kind      =  seesawKind;

% Distance of the feet from the center of the seesaw
seesaw.lFootDistance_y      =  0.11;
seesaw.rFootDistance_y      = -0.11;
seesaw.lFootDistance_x      = -0.01;
seesaw.rFootDistance_x      = -0.01;

% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01
seesaw.invInertia = eye(3)/seesaw.inertia;

switch seesaw.kind
    
    case 1 % Spherical seesaw
        seesaw.iota      = seesaw.mass*seesaw.invInertia;
        seesaw.invIota   = inv(seesaw.iota);
        
    case 2 % Semi cylidrical seesaw
        seesaw.iota      = [1;0;0]*transpose([1;0;0])*seesaw.mass*seesaw.invInertia;
        seesaw.invIota   = 0;
end

% adjust seesaw angle measurements (roll, pitch, yaw) [deg]
seesaw.offset            = [0; 0; 0];

%% World frame location
% Assumption: at time t=0, the seesaw is horizontal, and the left foot of
% the robot is at a given distance from the contact point of the seesaw
% with the ground ( = world frame origin).

% Rotation between the world and the left foot (= seesaw orientation) at
% time 0 assuming the left foot rigidly attached to the seesaw
w_R_lSole_0  = eye(3);

% Complete transformation
CONFIG.w_H_lSole_0  = computeTransFromLsole2World(w_R_lSole_0, seesaw);

model.seesaw = seesaw;

%% References
directionOfOscillation   = [0; 1; 0];
referenceParams          = [0.0 0.25];   %referenceParams(1) = amplitude of ascillations in meters referenceParams(2) = frequency of ascillations in hertz

noOscillationTime        = 0;  % If DEMO_LEFT_AND_RIGHT = true, the variable noOscillationTime is the time, in seconds, 
                               % that the robot waits before starting the left-and-right

%% Gains and references
gain.posturalProp      = diag([ 10 10 20,   10 10 10 8,   10 10 10 8,   30 30 20 20 0 0,   30 50 30 60 0 0 ]);                        
gain.posturalDamp      = gain.posturalProp*0;

gain.PAngularMomentum  = 0;
gain.DAngularMomentum  = 1;

gain.PCOM              = diag([ 10 50 20 ]);
gain.DCOM              = 2*sqrt(gain.PCOM)/10;
gain.ICOM              = diag([ 0 0 0 ]);

gain.P_SATURATION      = 0.30;

gain.seesawKP          = 0.1;
gain.seesawKD          = 2*sqrt(gain.seesawKP);

gain.seesawKP_passive  = 0.1;
gain.seesawKD_passive  = 2*sqrt(gain.seesawKP);
gain.seesawKLambda     = 0.5;
 
%% REGOLARIZATION TERMS
reg                    = struct;
reg.pinvTol            = 1e-7;
reg.pinvDamp           = 1e-1;
reg.pinvDampA          = 1e-4;
reg.HessianQP          = 1e-5;
reg.pinvDampVb         = 1e-3;
reg.impedances         = 0.1;
reg.dampings           = 0;

%% OTHER BALANCING CONTROLLERS (DIFFERENT FROM 1)
if  CONFIG.CONTROLKIND == 2   
    
    gain.posturalProp      = diag([ 10 10 20,   10 10 10 8,   10 10 10 8,   30 30 20 20 0 0,   30 50 30 60 0 0 ]);                   
    gain.posturalDamp      = gain.posturalProp*0;

    gain.PAngularMomentum  = 0;
    gain.DAngularMomentum  = 1;

    gain.PCOM              = diag([ 10 50 20 ])/10;
    gain.DCOM              = 2*sqrt(gain.PCOM)/40;
    gain.ICOM              = diag([ 0 0 0 ]);

    gain.P_SATURATION      = 0.30;

    gain.seesawKP          = 0.1/10;
    gain.seesawKD          = 2*sqrt(gain.seesawKP)/10;
    
elseif  CONFIG.CONTROLKIND == 3
    
    gain.posturalProp      = diag([ 10 10 20,   10 10 10 8,   10 10 10 8,   30 30 20 20 0 0,   30 50 30 60 0 0 ]);                         
    gain.posturalDamp      = gain.posturalProp*0;

    gain.PAngularMomentum  = 0;
    gain.DAngularMomentum  = 1;

    gain.PCOM              = diag([ 10 50 20 ])/5;
    gain.DCOM              = 2*sqrt(gain.PCOM)/20;
    gain.ICOM              = diag([ 0 0 0 ]);

    gain.P_SATURATION      = 0.30;

    gain.seesawKP          = 0.1;
    gain.seesawKD          = 2*sqrt(gain.seesawKP);
    
elseif  CONFIG.CONTROLKIND == 4   
    
    gain.posturalProp      = diag([ 10 10 20,   10 10 10 8,   10 10 10 8,   30 30 20 20 0 0,   30 50 30 60 0 0 ]);                       
    gain.posturalDamp      = gain.posturalProp*0;

    gain.PAngularMomentum  = 10;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    gain.PCOM              = diag([ 20 50 20 ])/2;
    gain.DCOM              = 2*sqrt(gain.PCOM)/10;
    gain.ICOM              = diag([ 0 0 0 ]);

    gain.P_SATURATION      = 0.30;

    gain.seesawKP          = 10;
    gain.seesawKD          = 20; 
    
    gain.seesawKP_passive  = 10;
    gain.seesawKD_passive  = 2*sqrt(gain.seesawKP)*2;
    gain.seesawKLambda     = 0.5;
    
    reg.pinvTol            = 1e-4;
    reg.pinvDamp           = 1e-2;
    reg.pinvDampA          = 1e-7;
    reg.pinvDampA          = 1e-4;
    reg.HessianQP          = 1e-5;
    reg.pinvDampVb         = 1e-2;

  elseif  CONFIG.CONTROLKIND == 5   
    
    gain.posturalProp      = diag([ 10 10 20,   10 10 10 8,   10 10 10 8,   30 30 20 20 0 0,   30 50 30 60 0 0 ]);                        
    gain.posturalDamp      = gain.posturalProp*0;

    gain.PAngularMomentum  = 1;
    gain.DAngularMomentum  = 1;

    gain.PCOM              = diag([ 10 50 20 ]);
    gain.DCOM              = 2*sqrt(gain.PCOM)/20;
    gain.ICOM              = diag([ 0 0 0 ]);

    gain.P_SATURATION      = 0.30;

    gain.seesawKP          = 0.1;
    gain.seesawKD          = 2*sqrt(gain.seesawKP);
    
    gain.seesawKP_passive  = 0.1;
    gain.seesawKD_passive  = 2*sqrt(gain.seesawKP);
    
    reg.pinvTol            = 1e-3;
    reg.pinvDamp           = 1e-2;
    reg.pinvDampA          = 1e-7;
    reg.pinvDampA          = 1e-4;
    reg.HessianQP          = 1e-5;
    
end

%% Friction cone parameters
numberOfPoints                = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                   % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient      = 1; %1/3;  
torsionalFrictionCoefficient  = 2/150;

% physical size of foot 
gain.footSize                 = [ -0.07  0.12 ;    % xMin, xMax
                                  -0.045 0.05 ];   % yMin, yMax     

fZmin                         = 20;
