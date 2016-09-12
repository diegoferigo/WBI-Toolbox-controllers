%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'YOGA')
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 

    reg.pinvDamp               = 1;
    reg.impedances             = 0.1;
    reg.dampings               = 0;
    reg.HessianQP              = 1e-7;

    sat.torque                 = 60;

    gain.footSize              = [ -0.07  0.12 ;    % xMin, xMax
                                   -0.045 0.05 ];   % yMin, yMax  
                   
    forceFrictionCoefficient     = 1/3;
    
    %Smoothing time for time varying impedances
    gain.SmoothingTimeGainScheduling              = 2;  

    %Smoothing time for time-varying constraints
    CONFIG.smoothingTimeTranDynamics  = 0.02;

    references.joints.smoothingTime    = 5 ;
    references.com.smoothingTime       = references.joints.smoothingTime;
    gain.SmoothingTimeImp              = references.joints.smoothingTime;  

    gain.PCOM     =    [50    50  10  % state ==  1  TWO FEET BALANCING
                        50    50  10  % state ==  2  COM TRANSITION TO LEFT 
                        50    50  10  % state ==  3  LEFT FOOT BALANCING
                        50    50  10  % state ==  4  YOGA LEFT FOOT 
                        50    50  10  % state ==  5  PREPARING FOR SWITCHING 
                        50    50  10  % state ==  6  LOOKING FOR CONTACT
                        50    50  10  % state ==  7  TRANSITION TO INITIAL POSITION 
                        50    50  10  % state ==  8  COM TRANSITION TO RIGHT FOOT
                        50    50  10  % state ==  9  RIGHT FOOT BALANCING
                        50    50  10  % state == 10  YOGA RIGHT FOOT 
                        50    50  10  % state == 11  PREPARING FOR SWITCHING 
                        50    50  10  % state == 12  LOOKING FOR CONTACT
                        50    50  10];% state == 13  TRANSITION TO INITIAL POSITION
    gain.PCOM  =  gain.PCOM;
    gain.ICOM  = gain.PCOM*0;
    gain.DCOM  = 2*sqrt(gain.PCOM)/20;

    gain.PAngularMomentum  = 0.25 ;
    gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

    % state ==  1  TWO FEET BALANCING
    % state ==  2  COM TRANSITION TO LEFT FOOT
    % state ==  3  LEFT FOOT BALANCING 
    % state ==  4  YOGA LEFT FOOT  
    % state ==  5  PREPARING FOR SWITCHING
    % state ==  6  LOOKING FOR CONTACT 
    
    % state ==  7  TRANSITION TO INITIAL POSITION
    
    % state ==  8  COM TRANSITION TO RIGHT FOOT
    % state ==  9  RIGHT FOOT BALANCING 
    % state == 10  YOGA RIGHT FOOT  
    % state == 11  PREPARING FOR SWITCHING
    % state == 12  LOOKING FOR CONTACT 
    % state == 13  TRANSITION TO INITIAL POSITION


    %                   %   TORSO  %%      LEFT ARM   %%      RIGHT ARM   %%         LEFT LEG            %%         RIGHT LEG           %% 
    gain.impedances  = [10   10   20, 30   30   20    20    100 100, 30   50   30    60    100 100  % state ==  1  TWO FEET BALANCING
                        10   10   20, 30   30   20    20    100 100, 30   50   30    60    100 100  % state ==  2  COM TRANSITION TO LEFT 
                        10   10   20, 30   50   30    60    100 100, 30   30   20    20    100 100  % state ==  3  LEFT FOOT BALANCING
                        30   30   30,100  200  100   400    100 100,100   50   30   100    100 100  % state ==  4  YOGA LEFT FOOT 
                        30   30   30,200  250   20    20     10  10,220  550  220   200     65 300  % state ==  5  PREPARING FOR SWITCHING 
                        30   30   30,100  350   20   200     10 100,220  550  220   200     65 300  % state ==  6  LOOKING FOR CONTACT
                        10   10   20, 30   50   60    30      5   5, 30   30   30    20      5   5  % state ==  7  TRANSITION TO INITIAL POSITION 
                        10   10   20, 30   50   60    30    100 100, 30   30   30    20    100 100  % state ==  8  COM TRANSITION TO RIGHT FOOT
                        10   10   20, 30   50   30    60    100 100, 30   30   20    20    100 100  % state ==  9  RIGHT FOOT BALANCING
                        30   30   30,100   50   30   100    100 100,100  200  100   400    100 100  % state == 10  YOGA RIGHT FOOT 
                        30   30   30,220  550  220   200     65 300,200  250   20    20     10  10  % state == 11  PREPARING FOR SWITCHING 
                        30   30   30,220  550  220   200     65 300,100  350   20   200     10 100  % state == 12  LOOKING FOR CONTACT
                        30   30   30,100   50   30   100    100 100,100  200   20   400    100 100];% state == 13  TRANSITION TO INITIAL POSITION
 end              
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                      
         
%% %%%%%%%%%%%%%%%%    FINITE STATE MACHINE SPECIFIC PARAMETERS
sm.skipYoga                      = false;
sm.demoOnlyRightFoot             = false;
sm.yogaAlsoOnRightFoot           = true;
sm.yogaInLoop                    = false;
sm.com.threshold                 = 0.01;
sm.wrench.thresholdContactOn     =  40;     % Force threshole above which contact is considered stable
sm.wrench.thresholdContactOff    = 80;     % Force threshole under which contact is considered off
sm.joints                        = struct;
sm.joints.thresholdNotInContact  =  5;      % Degrees
sm.joints.thresholdInContact     = 50;      % Degrees
sm.joints.pauseTimeLastPostureL  = 3;
sm.joints.pauseTimeLastPostureR  = 3;

sm.stateAt0                      = 1;

sm.DT                            = 1;
sm.waitingTimeAfterYoga          = 0;

sm.jointsSmoothingTimes          = [1;   %% state ==  1  TWO FEET BALANCING
                                         %%
                                    4;   %% state ==  2  COM TRANSITION TO LEFT FOOT
                                    5;   %% state ==  3  LEFT FOOT BALANCING 
                                    4;   %% state ==  4  YOGA LEFT FOOT
                                    5;   %% state ==  5  PREPARING FOR SWITCHING
                                    4;   %% state ==  6  LOOKING FOR CONTACT 
                                         %%
                                    6;   %% state ==  7  TRANSITION INIT POSITION
                                         %%
                                    4;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                                    5;   %% state ==  9  RIGHT FOOT BALANCING 
                                    4;   %% state == 10  YOGA RIGHT FOOT
                                    5;   %% state == 11  PREPARING FOR SWITCHING
                                    4;   %% state == 12  LOOKING FOR CONTACT 
                                         %%
                                    6];  %% state == 13  TRANSITION INIT POSITION

sm.com.states      = [0.0,  0.01,0.0;   %% state ==  1  TWO FEET BALANCING NOT USED
                      0.0,  0.00,0.0;   %% state ==  2  COM TRANSITION TO LEFT FOOT: THIS REFERENCE IS USED AS A DELTA W.R.T. THE POSITION OF THE LEFT FOOT
                      0.0,  0.00,0.0;   %% state ==  3  LEFT FOOT BALANCING 
                      0.0,  0.01,0.0;   %% state ==  4  YOGA LEFT FOOT
                      0.0,  0.00,0.0;   %% state ==  5  PREPARING FOR SWITCHING
                      0.0, -0.09,0.0;   %% state ==  6  LOOKING FOR CONTACT 
                      0.0,  0.00,0.0;   %% state ==  7  TRANSITION INIT POSITION: DELTAS W.R.T. CoM_0
                      % FROM NOW ON, THE REFERENCE ARE ALWAYS DELTAS W.R.T.
                      % THE POSITION OF THE RIGHT FOOT
                      0.0, -0.00,0.0;   %% state ==  8  COM TRANSITION TO RIGHT FOOT
                      0.0,  0.00,0.0;   %% state ==  9  RIGHT FOOT BALANCING 
                      0.0, -0.01,0.0;   %% state == 10  YOGA RIGHT FOOT
                      0.0, -0.00,0.0;   %% state == 11  PREPARING FOR SWITCHING
                      0.0,  0.09,0.0;   %% state == 12  LOOKING FOR CONTACT 
                      0.0,  0.00,0.0];  %% state == 13  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
sm.tBalancing      = 1;%0.5;


sm.joints.states = [[0.0864,0.0258,0.0152, ...                          %% state == 1  TWO FEET BALANCING, THIS REFERENCE IS IGNORED 
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 2  COM TRANSITION TO LEFT 
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     %  
                    [0.0864,0.0258,0.0152, ...                          %% state == 3  LEFT FOOT BALANCING
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 4  YOGA LEFT FOOT, THIS REFERENCE IS IGNORED
                     0.0522,-0.2582,0.0014,-0.2129,-0.0944,0.1937,...   %
                     0.0128,0.4367,0.0093,-0.1585,-0.0725,-0.2931];     %
                    [-0.0348,0.0779,0.0429, ...                         %% state == 5  PREPARING FOR SWITCHING
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630, ...  %  
                     0.0005,0.0793,-0.0014,-0.0051    0.0073   -0.1151];%                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 6  LOOKING FOR CONTACT
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     %   
                     zeros(1,ROBOT_DOF);                                %% state == 7  TRANSITION INIT POSITION: THIS REFERENCE IS IGNORED
                    [0.0864,0.0258,0.0152, ...                          %% state == 8  COM TRANSITION TO RIGHT FOOT
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369,...   %
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277];     % 
                    [0.0864,0.0258,0.0152, ...                          %% state == 9  RIGHT FOOT BALANCING
                     0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ...  %  
                     -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];     %  
                     zeros(1,ROBOT_DOF);                                %% state == 10  YOGA RIGHT FOOT, THIS REFERENCE IS IGNORED  
                    [-0.0348,0.0779,0.0429, ...                         %% state == 11  PREPARING FOR SWITCHING
                      0.0005,0.0793,-0.0014,-0.0051,0.0073,-0.1151, ... %  
                      -0.0015,-0.1109,-0.0001,0.0003,0.0160,0.1630];    %                                  %
                    [0.0864,0.0258,0.0152, ...                          %% state == 12  LOOKING FOR CONTACT
                     -0.0026,0.0225,0.0093,-0.0020,0.0027,-0.0277,...   %
                     0.0107,-0.0741,-0.0001,-0.0120,0.0252,0.1369];     %   
                    zeros(1,ROBOT_DOF)];                                %% state == 13  BALANCING TWO FEET, THIS REFERENCE IS IGNORED                     

 
q1 =        [-0.0790,0.2279, 0.4519, ...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044, 0.0700, ...
              0.3484,0.4008,-0.0004,-0.3672,-0.0530,-0.0875];

q2 =        [-0.0790,0.2279, 0.4519, ...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ... 
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
q3 =        [-0.0852,-0.4273,0.0821,...
              0.2092,0.2960, 0.0006,-0.1741,-0.1044,0.0700, ...
              0.3714,0.9599, 1.3253,-1.6594, 0.6374,-0.0614];
q4 =        [-0.0852,-0.4273,0.0821,...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q5 =        [-0.0790,0.2279, 0.4519, ...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q6 =        [-0.0852,-0.4273,0.0821,...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
          
q7 =        [-0.0852,-0.4273,0.0821,...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253, -1.6217, 0.6374,-0.0614];
          
q8 =        [-0.0852,-0.4273,0.0821,...
              0.2092, 0.6473,0.0006,-0.1741,-0.1044, 0.0700,...
              0.3514, 1.3107,1.3253,-0.0189, 0.6374,-0.0614];
%           
%              -0.2793,1.4312,-0.5585,1.5708, ...
%              -0.2793,1.4312,-0.5585,1.5708, ...
          
q9 =        [-0.0179    0.2145   -0.0016,...
               0.3835    0.3846   -0.0001   -0.2973   -0.0852    0.0311  ,...
              0.2091    0.2971    0.0001   -0.1754   -0.0940,0.0597];

          
  
sm.joints.pointsL =[ 0,                            q1;
                     1*sm.jointsSmoothingTimes(4),q2;
                     2*sm.jointsSmoothingTimes(4),q3;
                     3*sm.jointsSmoothingTimes(4),q4;
                     4*sm.jointsSmoothingTimes(4),q5;
                     5*sm.jointsSmoothingTimes(4),q6;
                     6*sm.jointsSmoothingTimes(4),q7;
                     7*sm.jointsSmoothingTimes(4),q8];
                 
% sm.joints.pointsL = [references.joints.smoothingTime,sm.joints.states(5,:)];

sm.joints.pointsR = sm.joints.pointsL;

					 
for i = 1:size(sm.joints.pointsR,1)				
	sm.joints.pointsR(i,2:4)          = [sm.joints.pointsR(i,2) -sm.joints.pointsR(i,3) -sm.joints.pointsR(i,4)];
	
	
	rightLeg                          =  sm.joints.pointsR(i,end-5:end);
	sm.joints.pointsR(i,end-5:end)    =  sm.joints.pointsR(i,end-11:end-6);
	sm.joints.pointsR(i,end-11:end-6) =  rightLeg;
end	 


clear q1 q2 q3 q4;

sm.joints.smoothingTime    = references.joints.smoothingTime;
sm.com.smoothingTime       = references.com.smoothingTime;
