clc
clear variables

%% Robot name
% setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');
setenv('YARP_ROBOT_NAME','iCubGenova04');

% simulation time [s]
CONFIG.TS                  = 0.01;
% QP solver
CONFIG.USE_QP_SOLVER       = true;
% Visualization tool
CONFIG.SCOPES              = true;
% Used for the integral of angular momentum
LEFT_RIGHT_FOOT_IN_CONTACT = [1;1];

%% Balancing controller configuration parameters
CONFIG.CONTROLKIND         = 1; % either 1,2,3,4,5
CONFIG.USE_PASSIVE_CONTROL = 0; 
CONFIG.CONSIDER_SEESAW_DYN = 1;

%% State estimation configuration parameters

% Robot IMU related
CONFIG.CORRECT_NECK_IMU              = true;
CONFIG.USE_IMU4EST_BASE              = true;
CONFIG.USE_IMU_ROBOT_4_SEESAW_ORIENT = true;
CONFIG.YAW_IMU_FILTER                = true;
CONFIG.PITCH_IMU_FILTER              = true;

% Seesaw IMU related
CONFIG.FILTER_SEESAW_ANGVEL          = false;
CONFIG.USE_SEESAW_ANGVEL             = true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DO NOT MODIFY THE FOLLOWING VARIABLES, THEY ARE AUTOMATICALLY 
%% CHANGED WHEN SIMULATING THE ROBOT ON GAZEBO, i.e. YARP_ROBOT_NAME=icubGazeboSim
addpath('../../utilityMatlabFunctions/')
addpath('./src')

WBT_modelName        = 'matlabTorqueBalancingSeesaw';

CONFIG.ON_GAZEBO     = false;

PORTS.IMU            = '/icub/inertial';
PORTS.IMUSEESAW      = '/seesaw';
PORTS.NECK           = '/icub/head/state:o';

% Choose seesaw shape
seesawKind           = 2; % 1 = spherical; 2 = semi-cylinder

CONFIG.SEESAW_WITH_VERTICAL_BORDER  = 1;         % new seesaw kind
CONFIG.SEESAW_HAS_IMU               = 1;         % seesaw has mounted IMU

run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/initGains.m')); 
[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.footSize,fZmin);
