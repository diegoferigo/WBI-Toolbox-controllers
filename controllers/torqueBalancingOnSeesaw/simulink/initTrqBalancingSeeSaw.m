clc
clear variables

% Robot names
setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
% setenv('YARP_ROBOT_NAME','iCubGenova04');

localName                  = 'seesawBalancingController';
Ts                         = 0.01;

% Choose control mode
CONFIG.CONTROLKIND         = 1;
CONFIG.USE_PASSIVE_CONTROL = 0; 

CONFIG.USE_QP_SOLVER       = true;
CONFIG.SCOPES              = true;
CONFIG.CORRECT_NECK_IMU    = true;
CONFIG.TS                  = 0.01;

CONFIG.CONSIDERSEESAWDYN   = 1;

% Used for the new integral of angular momentum estimation
LEFT_RIGHT_FOOT_IN_CONTACT = [1;1];

CONFIG.USE_IMU4EST_BASE    = true;
CONFIG.USE_IMU_ROBOT_4_SEESAW_ORIENT = true;
CONFIG.FILTER_SEESAW_ANGVEL = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
WBT_modelName        = 'matlabTorqueBalancingSeesaw';

addpath('../../utilityMatlabFunctions/')
addpath('./src')

CONFIG.ON_GAZEBO     = false;

PORTS.IMU            = '/icub/inertial';
PORTS.IMUSEESAW      = '/seesaw';
PORTS.NECK           = '/icub/head/state:o';

% Choose seesaw shape
seesawKind           = 2;

CONFIG.SEESAW_WITH_VERTICAL_BORDER  = 1;         % new seesaw kind
CONFIG.SEESAW_HAS_IMU               = 1;         % seesaw has mounted IMU
CONFIG.USE_PASSIVE_CONTROL          = 0;         % passive controller for seesaw orientation

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/initGains.m')); 
[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);

CONFIG.USE_ROBOT_IMU4SEESAW = true;
CONFIG.YAW_IMU_FILTER       = true;
CONFIG.PITCH_IMU_FILTER     = true;
