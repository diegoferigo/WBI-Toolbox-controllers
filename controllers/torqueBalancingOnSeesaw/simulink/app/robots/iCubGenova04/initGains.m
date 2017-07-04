ROBOT_DOF              = 23;
WBT_wbiList            = '(torso_pitch,torso_roll,torso_yaw,l_shoulder_pitch,l_shoulder_roll,l_shoulder_yaw,l_elbow,r_shoulder_pitch,r_shoulder_roll,r_shoulder_yaw,r_elbow,l_hip_pitch,l_hip_roll,l_hip_yaw,l_knee,l_ankle_pitch,l_ankle_roll,r_hip_pitch,r_hip_roll,r_hip_yaw,r_knee,r_ankle_pitch,r_ankle_roll)';
sat.torque             = 12;

%% Seesaw parameters
seesaw                 = struct;

% Height of the seesaw
%  ___________
% |           |  ---> vertical border
%  *         *   | 
%   *      *     | hArc
%     *  *       |
seesaw.hArc            = 0.11;
seesaw.metalPlate      = 0.004;
seesaw.verticalBorder  = 0.046;
seesaw.h               = seesaw.hArc + seesaw.verticalBorder;

% Radius of the seesaw
seesaw.rho             = 0.362;

% Height of seesaw CoM
shiftCoM               = 0.02;
seesaw.CoM_z           = seesaw.h - shiftCoM;

% Seesaw inertia and mass (IMU is considered in the mass but not for the inertia)
seesaw.mass            = 6.2;

% INERTIA TENSOR:
% Ixx Ixy Ixz 7.6698599e-02 0.0000000e+00 0.0000000e+00
% Iyx Iyy Iyz 0.0000000e+00 3.7876787e-02 0.0000000e+00
% Izx Izy Izz 0.0000000e+00 0.0000000e+00 1.0893139e-01
seesaw.inertia         = diag([7.6698599e-02, 3.7876787e-02, 1.0893139e-01]);

% Distance between the center of rotation and the center of mass
seesaw.delta           = seesaw.rho - seesaw.CoM_z;

% Distance (vertical) between the seesaw/feet contact plane and the seesaw CoM
seesaw.top             = (seesaw.h + seesaw.metalPlate) - seesaw.CoM_z;

% Distance (XY) of the feet from the center of the seesaw (or seesaw xy CoM)
seesaw.lFootDistance_y =  0.11;
seesaw.rFootDistance_y = -0.11;
seesaw.lFootDistance_x = -0.01;
seesaw.rFootDistance_x = -0.01;

% Distance between the seesaw CoM and the feet
seesaw.s_sl            = [seesaw.lFootDistance_x;seesaw.lFootDistance_y; seesaw.top];
seesaw.s_sr            = [seesaw.rFootDistance_x;seesaw.rFootDistance_y; seesaw.top];

if strcmp(FRAMES.fixedLink,'l_sole')
    
    % Link fixed with the seesaw
    seesaw.s_sFixed    = seesaw.s_sl;
    
elseif strcmp(FRAMES.fixedLink,'r_sole')
    
    seesaw.s_sFixed    = seesaw.s_sr;
    
else
    
    error('The robot frame which is assumed to be fixed w.r.t. the seesaw is not valid!')    
end

% Adjust seesaw angle measurements (roll, pitch, yaw) [deg]
seesaw.offset          = [0; 0; 0];

% Relative rotation between world frame and IMU seesaw world frame
addpath('../../../../../utilityMatlabFunctions/');
seesaw.w_R_wImu = rotx(pi)*rotz(-44/180*pi); 

%% Filter seesaw measurements

% Both velocity and orientation filters compute the average of a
% given number of samples coming from the IMU seesaw measurements. The
% number of samples is the order of the filter.

% Select the order of the filter for seesaw orientation and ang velocity
seesaw.positionFilterOrder = 5;
seesaw.velocityFilterOrder = 5;

%% References for CoM trajectory
directionOfOscillation     = [0; 1; 0];
referenceParams            = [0.0 0.25]; % referenceParams(1) = amplitude of ascillations in meters; referenceParams(2) = frequency of ascillations in Hertz
noOscillationTime          = 0; % the variable noOscillationTime is the time, in seconds, that the robot waits before starting moving the CoM left-and-right

%% Gains and regularization terms (for all different controllers)
if CONFIG.CONTROL_KIND == 1
    
    % gain on seesaw angular velocity
    gain.KthetaDot         = 0; % NOT USED
    gain.Ktheta            = 0; % NOT USED

    % By default these values are used by CONTROL_KIND 1
    gain.posturalProp      = diag([100 10 200,   100 100 100 80,   100 100 100 80,   600 60 600 600 100 10,   600 60 600 600 100 10])/10;                    
    gain.posturalDamp      = 2*sqrt(gain.posturalProp)*0;

    gain.PAngularMomentum  = 2/20;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum)/5;

    gain.PCOM              = diag([ 200 20 200])/10;
    gain.DCOM              = 2*sqrt(gain.PCOM)/20;

    % Saturate the CoM position error
    gain.P_SATURATION      = 0.30;
    
    % Regularization terms
    reg                    = struct;
    reg.pinvDamp           = 1;
    reg.HessianQP          = 1e-7;
    reg.pinvTol            = 1e-3;
    reg.pinvTolVb          = 1e-7;
    
elseif CONFIG.CONTROL_KIND == 2
    
    % gain on seesaw angular velocity
    gain.Ktheta            = 1;
    gain.KthetaDot         = 1;

    % By default these values are used by CONTROL_KIND 1
    gain.posturalProp      = diag([100 50 200,   100 100 100 80,   100 100 100 80,   600 60 600 600 100 10,   600 60 600 600 100 10])/10;                        
    gain.posturalDamp      = 2*sqrt(gain.posturalProp)*0;

    gain.PAngularMomentum  = 1;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum)/10;

    gain.PCOM              = diag([ 200 10 200 ])/10;
    gain.DCOM              = 2*sqrt(gain.PCOM)/10;

    % Saturate the CoM position error
    gain.P_SATURATION      = 0.3;
     
end

%% Friction cone parameters
numberOfPoints         = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                            % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 
% Friction parameters
forceFrictionCoefficient      = 1;   
torsionalFrictionCoefficient  = 2/150;

% Physical size of foot 
gain.footSize                 = [ -0.07  0.12 ;    % xMin, xMax
                                  -0.045 0.05 ];   % yMin, yMax     
% Minimal normal force
fZmin                         = 20;
