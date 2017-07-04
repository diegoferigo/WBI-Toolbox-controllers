clc
clear variables

%% GENERIC CONFIGURATION PARAMETERS

% Robot name
setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','iCubGenova04');

% Simulation time step (fixed) [s]
CONFIG.TS                  = 0.01;

% Activate visualization
CONFIG.SCOPES              = true;

% Use QP solver for applying friction cones and unilateral constraints to
% the contact frces at feet
CONFIG.USE_QP_SOLVER       = true;

%% CONFIGURATION PARAMETERS FOR BALANCING CONTROLLER

% Choose between different balancing controllers. 
% 
% CONTROL_KIND 1 = control objective: Centroidal momentum dynamics of the
%                  robot only;
%
% CONTROL_KIND 2 = control objective: seesaw angular momentum in x direction;
%                  obtained by controlling the centroidal momentum dynamics 
%                  0of the robot;
%
CONFIG.CONTROL_KIND = 2; 

%% CONFIGURATION PARAMETERS FOR STATE ESTIMATION

% Select the link frame that is fixed w.r.t. the seesaw. Only two frames can be
% selected: l_sole and r_sole
FRAMES.fixedLink = 'l_sole';

PORTS.WBDT_LEFTLEG_EE  = '/wholeBodyDynamicsTree/left_leg/cartesianEndEffectorWrench:o';
PORTS.WBDT_RIGHTLEG_EE = '/wholeBodyDynamicsTree/right_leg/cartesianEndEffectorWrench:o';


%% Configuration options for robot IMU only

% IMU robot frame name
FRAMES.imu = 'imu_frame';

% If True, it will not assume that the neck position is [0;0;0], but the
% neck angles are taken into account instead for correcting IMU measurements
CONFIG.CORRECT_IMU_WITH_NECK         = false;

% Given the particular shape of the seesaw, the seesaw yaw and pitch should
% be zero. If these filters are True, they will force the seesaw pitch and
% yaw to be zero
CONFIG.YAW_IMU_FILTER                = true;
CONFIG.PITCH_IMU_FILTER              = true;

% If true, seesaw orientation and/or angular velocity are evaluated using
% the robot IMU. Otherwise, the seesaw IMU is used instead
CONFIG.USE_IMU_ROBOT_4_SEESAW_ORIENT = true;
CONFIG.USE_IMU_ROBOT_4_SEESAW_ANGVEL = true;

%% Configuration options for seesaw IMU only

% Apply a filter on the data coming from the sesaw IMU. Advantage: they
% remove the noise coming from the measurements. Disadvantage: they
% inroduce a delay
CONFIG.FILTER_SEESAW_ORIENT          = true;
CONFIG.FILTER_SEESAW_ANGVEL          = true;

% As before, given the shape of the seesaw, y-z angular velocity should be zero. This 
% option is to force omega_y and omega_z to be zero
CONFIG.REMOVE_SEESAW_YZ_ANGVEL       = false;

%% LOADING GAINS AND PARAMETERS FOR THE SPECIFIC ROBOT

% WARNING: DO NOT MODIFY THESE PARAMETERS. They are automatically updated
% if required (e.g. in simulation, PORTS.IMU is automatically changed in 
% 'icubSim/inertial)

% Ports to read IMU measurements and neck positions
PORTS.IMU            = '/icub/inertial';
PORTS.IMU_SEESAW     = '/seesaw';
PORTS.NECK           = '/icub/head/state:o';

% This information is used for evaluating the integral of the angular
% momentum error
LEFT_RIGHT_FOOT_IN_CONTACT = [1;1];

% Add path to the required matlab functions
addpath('../../utilityMatlabFunctions/')
addpath('./src')

% Enforce information for WBToolbox
WBT_modelName        = 'matlabTorqueBalancingSeesaw';

% By default, it is assumed the controller is for the real iCub
CONFIG.ON_GAZEBO     = false;

% Run the functions containing initial control gains and all the
% information related to the seesaw
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/initGains.m'));

% Friction and unilateral constraints
[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

% Transformation between world frame and fixed link at time t=0. 
% Assumption: at time t=0, the seesaw is horizontal, and the fixed link of
% the robot is at a given distance from the contact point of the seesaw
% with the ground ( = world frame origin).

% Rotation between the world and the fixed link (= seesaw orientation) at
% time 0 assuming the fixed link rigidly attached to the seesaw
w_R_fixedLink_0        = eye(3);

% Complete transformation
seesaw.w_H_fixedLink_0 = computeTransFromFixedLinkToWorld(w_R_fixedLink_0,seesaw);
