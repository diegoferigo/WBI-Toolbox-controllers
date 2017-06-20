function [comError, fNoQp, f_HDot, NA, tauModel, Sigmaf_HDot, SigmaNA, ...
          HessianMatrixQP2Feet, gradientQP2Feet, ConstraintsMatrixQP2Feet, bVectorConstraintsQp2Feet, saturate_pos] = ...
          balancingControlSeesaw ...
                                 (t, state_0, state, ConstraintsMatrix, bVectorConstraints, gain, reg, ...
                                  x_dx_ddx_CoMRef, qjRef, seesaw, ROBOT_DOF, J, J_CoM, Jdot_nu, M, genBiasForces, ...
                                  w_p_com_robot, w_H_lSole, w_H_rSole, H, intHw, CONFIG)

 % BALANCING_CONTROL_SEESAW 
   persistent w_p_com_total_0 

   % Configuration parameters
   robotDoFs       = size(ROBOT_DOF,1);  
   gravityAcc      = [0; 0; -9.81];

   [seesaw_pose, seesaw_vel, robotPos, robotVel]         = state_partitioning(state, robotDoFs);
   [seesaw_pose_0, ~, ~, ~] = state_partitioning(state_0, robotDoFs);
   
    qj             = robotPos(8:end);
    
    w_R_seesaw     = rotationFromQuaternion(seesaw_pose(1:4));
    w_R_seesaw_0   = rotationFromQuaternion(seesaw_pose_0(1:4));
    
    s_omega_seesaw = seesaw_vel(1:3);
    w_omega_seesaw = w_R_seesaw * s_omega_seesaw;

    % Dynamics quantities
    m_robot        = M(1,1);
    m_seesaw       = seesaw.mass;
    m_total        = m_robot + m_seesaw;
    
    % The mass matrix is partitioned as:
    %
    % M = [ Mb   Mbj
    %       Mbj' Mj ];
    %
    % Mb \in R^{6x6}, Mbj \in R^{6x6+nDof}, Mj \in R^{nDofxnDof}
    %
    Mb              = M(1:6,1:6);
    Mbj             = M(1:6,7:end);
    Mj              = M(7:end,7:end);

    St              = [zeros(6,robotDoFs);
                       eye(robotDoFs,robotDoFs)];
  
    gravityWrench   = [M(1,1)*gravityAcc; zeros(3,1)];

    w_p_lSole       = w_H_lSole(1:3,4);
    w_p_rSole       = w_H_rSole(1:3,4);
    w_R_lSole       = w_H_lSole(1:3,1:3);
    w_R_rSole       = w_H_lSole(1:3,1:3);

    Lambda          = J / M * St;
    LambdaPinv      = pinvDamped(Lambda, reg.pinvDamp); 
    
    NullLambda      = LambdaPinv * Lambda;
    NullLambda      = eye(size(NullLambda)) - NullLambda;
    
    e1              = [1;0;0];
    e2              = [0;1;0];
    e3              = [0;0;1];
    
    %% Vectors and rotations
    
    % vector between CoM and seesaw contact point w.r.t. world frame
    r_w            = seesaw.delta * w_R_seesaw * e3 - seesaw.rho * e3 ; 
    
    % rotation between seesaw and world in seesaw frame
    seesaw_R_w     = transpose(w_R_seesaw);
    
    % gravity in the seesaw frame
    g_seesaw       = seesaw_R_w * gravityAcc; 
    
    % r_w in seesaw frame
    r_s            = seesaw_R_w * r_w;  
    
    % derivative of r_s
    dr_s           = seesaw.rho * Sf(s_omega_seesaw) * seesaw_R_w * e3; 

    % distance between left/right foot and the seesaw frame in seesaw
    % coordinates
    s_p_lSole      = [seesaw.lFootDistance_x; seesaw.lFootDistance_y; seesaw.top];
    s_p_rSole      = [seesaw.lFootDistance_x; seesaw.lFootDistance_y; seesaw.top];

    % same distance in world frame
    w_s_lSole      = w_R_seesaw * s_p_lSole;
    w_s_rSole      = w_R_seesaw * s_p_rSole;

    w_p_com_seesaw = w_p_lSole - w_s_lSole;
        
    As             = [ eye(3),        zeros(3), eye(3),        zeros(3); ...
                       Sf(w_s_lSole), eye(3),   Sf(w_s_rSole), eye(3)];
               
    Delta          = [Sf(r_s - s_p_lSole); eye(3); Sf(r_s - s_p_rSole); eye(3)];
    DeltaDot       = [eye(3); zeros(3);  eye(3);  zeros(3)] * Sf(dr_s);
    
    % CoM velocity (robot)
    w_v_com_robot  = J_CoM(1:3,:) * robotVel;

    %% Refecences for CoM and seesaw angle (theta, rotation along x axis)
    
    % theta_ref amplitude and frequency
    amplitude              = 0 * pi/180;
    frequency              = 0.5;
    theta_0                = atan2(w_R_seesaw_0(3,2), w_R_seesaw_0(2,2));

    theta_seesaw_ref       = theta_0 + amplitude * sin(2*pi*frequency*t); 
    thetaDot_seesaw_ref    = amplitude * frequency * 2*pi * cos(2*pi*frequency*t);
  
    % theta_seesaw         = atan2(w_R_seesaw(3,2),w_R_seesaw(2,2));
   
    posCoM_desired         = x_dx_ddx_CoMRef(:,1); 
   
    saturate_pos           = saturate(gain.PCOM * (posCoM_desired - w_p_com_robot(1:3)), ...
                                     -gain.P_SATURATION, gain.P_SATURATION);
   
    ddx_CoMStar            = x_dx_ddx_CoMRef(:,3) + ...
                             saturate_pos + ...
                             gain.DCOM * (x_dx_ddx_CoMRef(:,2) - w_v_com_robot);

    Hdot_robot_desired     = [ m_robot*ddx_CoMStar; 
                              -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw];

    CentroidalMatr         = [eye(3), zeros(3), eye(3), zeros(3);
                              Sf(w_p_lSole-w_p_com_robot), eye(3), Sf(w_p_rSole-w_p_com_robot), eye(3)];
                  
    %% Seesaw parameters
    Omega_1                = zeros(3);
    Omega_2                = zeros(3);                     
    Omega_2Bar             = zeros(1,6);
    lambda_2               = 0;
    
    if seesaw.kind == 1    % Spherical seesaw
        
        Theta          = eye(3) + Sf(r_s) * seesaw.iota * transpose(Sf(r_s));
        Iota_r         = eye(3) + seesaw.iota * norm(r_s)^2;
        Omega_0        = seesaw.iota + (1/det(Theta)) * seesaw.iota * Sf(r_s) * Iota_r * Sf(r_s) * seesaw.iota;
        Omega_1        = (1/det(Theta)) * seesaw.iota * Theta * Sf(r_s) * (Sf(dr_s) * s_omega_seesaw - Sf(s_omega_seesaw)^2 * r_s - g_seesaw) ...
                         -Omega_0 * Sf(s_omega_seesaw) * seesaw.invIota * s_omega_seesaw;
        Omega_2        = [-seesaw.iota * Theta*Sf(r_s)/det(Theta),Omega_0] * (1/seesaw.mass);
        
    elseif seesaw.kind == 2 % Semicylindrical seesaw
            
        lambda1        = seesaw.delta*seesaw.iota(1,1)*(seesaw.rho*s_omega_seesaw(1)^2 -gravityAcc(3))/(1 + seesaw.iota(1,1) * norm(r_s)^2);
        lambda_2       = seesaw.iota(1,1)/(seesaw.mass*(1+seesaw.iota(1,1)*norm(r_s)^2));
        Omega_2Bar     = [(seesaw.delta*w_R_seesaw*e2-seesaw.rho*e2)',e1'];
        Omega_1        = -e1*lambda1*w_R_seesaw(3,2);
        Omega_2        =  e1*lambda_2*Omega_2Bar*blkdiag(w_R_seesaw, w_R_seesaw);
    end
    
    comError           = x_dx_ddx_CoMRef(:,1)  - w_p_com_robot(1:3);
    
    %% BALANCING CONTROLLERS
    if CONFIG.CONTROLKIND == 1
       
        % Only robot centroidal momentum is controlled as primary task
        A              = CentroidalMatr;
        pinvA          = pinv(A, reg.pinvTol);
        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
        f_HDot         = pinvA* (Hdot_robot_desired - gravityWrench);

    elseif CONFIG.CONTROLKIND == 2

        % Robot centroidal momentum + seesaw as primary task
        A              =  [CentroidalMatr;
                          -lambda_2*Omega_2Bar*As];

        pinvA          = pinv(A, reg.pinvTol);
        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
        
        theta          = atan2(w_R_seesaw(3,2), w_R_seesaw(2,2)) * 180/pi;
        thetaDot       = w_omega_seesaw(1) * 180/pi;
        
        desiredDyn     = [ Hdot_robot_desired;
                          -gain.seesawKP*(theta-theta_seesaw_ref*180/pi)-gain.seesawKD*thetaDot];
        
        f_HDot         = pinvA * (desiredDyn - [gravityWrench;0]);
        
   elseif CONFIG.CONTROLKIND == 3
       
        % Robot angular momentum + total linear momentum of the system as primary task
        
        % total CoM
        w_p_com_total  =  (m_seesaw * w_p_com_seesaw + m_robot * w_p_com_robot)/(m_robot + m_seesaw);
        
        if isempty(w_p_com_total_0)
            w_p_com_total_0 = w_p_com_total;
        end
           
        % total CoM velocity
        w_v_com_seesaw =  Sf(r_w) * w_omega_seesaw;
        w_v_com_total  =  (m_seesaw * w_v_com_seesaw + m_robot * w_v_com_robot)/(m_robot + m_seesaw);

        Theta          =  eye(3) + Sf(r_s) * seesaw.iota * transpose(Sf(r_s));

        w_F_c1         =  (w_R_seesaw/Theta)*(m_seesaw * Sf(dr_s) * s_omega_seesaw -m_seesaw * g_seesaw -m_seesaw * Sf(s_omega_seesaw)^2 * r_s);
     
        AL             = -(w_R_seesaw/Theta)*[-seesaw_R_w, m_seesaw*Sf(r_s)*seesaw.iota*seesaw_R_w]*As;
        
        A              =  [AL;
                           CentroidalMatr(4:6,:)];

        pinvA          = pinv(A, reg.pinvTol);

        NA             = pinvA * A;
        NA             = eye(size(NA)) - NA;
        
        desiredDyn     = Hdot_robot_desired;
        
        desiredDyn(1:3)= -m_total*(gain.PCOM * (w_p_com_total-w_p_com_total_0) ...
                                  +gain.DCOM *  w_v_com_total);
        
        f_HDot         = pinvA* (desiredDyn - [m_total*gravityAcc+w_F_c1;zeros(3,1)]);
        
        comError       = w_p_com_total_0  - w_p_com_total;
        
    elseif CONFIG.CONTROLKIND == 4

        % Control robot dynamics first, and the seesaw dynamics in the null
        % space
        A_robot        =  CentroidalMatr;
        A_seesaw       = -lambda_2*Omega_2Bar*As;

        pinvAr         = pinv(A_robot, reg.pinvTol);
        pinvAseesaw    = pinv(A_seesaw, reg.pinvTol);

        NAr            = pinvAr * A_robot;
        NAr            = eye(size(NAr)) - NAr;
        NAs            = pinvAseesaw * A_seesaw;
        NAs            = eye(size(NAs)) - NAs;

        NA             = NAr*NAs;
        
        theta          = atan2(w_R_seesaw(3,2),w_R_seesaw(2,2)) * 180/pi;
        thetaDot       = w_omega_seesaw(1) * 180/pi;
       
        desiredDyn_robot    =  Hdot_robot_desired; 
        desiredDyn_seesaw   = -gain.seesawKP*(theta-theta_seesaw_ref*180/pi)-gain.seesawKD*(thetaDot-thetaDot_seesaw_ref*180/pi);
        
        f_HDot_robot        = pinvAr * (desiredDyn_robot - gravityWrench);
        
        if CONFIG.USE_PASSIVE_CONTROL == 1 
            
            f_HDot_seesaw   = -gain.seesawKP_passive*transpose(A_seesaw*NAr)*(theta-theta_seesaw_ref*180/pi)...
                              -gain.seesawKD_passive*transpose(A_seesaw*NAr)*(thetaDot-thetaDot_seesaw_ref*180/pi);
        else
            f_HDot_seesaw   = pinv(A_seesaw*NAr, reg.pinvTol)*(desiredDyn_seesaw -A_seesaw*f_HDot_robot);
        end
        
        f_HDot              = f_HDot_robot + NAr*f_HDot_seesaw;
        
   elseif CONFIG.CONTROLKIND == 5

       % Control seesaw dynamics first, and the robot dynamics in the null
       % space
       A_robot          =  CentroidalMatr;
       A_seesaw         = -lambda_2*Omega_2Bar*As;

       pinvAr          = pinv(A_robot, reg.pinvTol);
       pinvAseesaw     = pinv(A_seesaw, reg.pinvTol);

       NAr             = pinvAr * A_robot;
       NAr             = eye(size(NAr)) - NAr;
       NAs             = pinvAseesaw * A_seesaw;
       NAs             = eye(size(NAs)) - NAs;

       NA              = NAs*NAr;
        
       theta           = atan2(w_R_seesaw(3,2),w_R_seesaw(2,2)) * 180/pi;
       thetaDot        = w_omega_seesaw(1) * 180/pi;
        
       desiredDyn_robot    =  Hdot_robot_desired; 
       desiredDyn_seesaw   = -gain.seesawKP*(theta-theta_seesaw_ref*180/pi)-gain.seesawKD*(thetaDot-thetaDot_seesaw_ref*180/pi);
        
        if CONFIG.USE_PASSIVE_CONTROL
            f_HDot_seesaw  = -gain.seesawKP_passive*transpose(A_seesaw)*(theta-theta_seesaw_ref*180/pi)...
                             -gain.seesawKD_passive*transpose(A_seesaw)*(thetaDot-thetaDot_seesaw_ref*180/pi);
        else
            f_HDot_seesaw  = pinvAseesaw*desiredDyn_seesaw;
        end
        
        f_HDot_robot    = pinv(A_robot*NAs,0.0001)*(desiredDyn_robot - gravityWrench -A_robot*f_HDot_seesaw);
        f_HDot          = f_HDot_seesaw + NAs*f_HDot_robot;

    else
        NA              = zeros(12);
        f_HDot          = zeros(12,1); 
    end
    
    %% Other parameters
    F                  = J / M * transpose(J) + ...
                         CONFIG.CONSIDER_SEESAW_DYN*blkdiag(w_R_seesaw,w_R_seesaw,w_R_seesaw,w_R_seesaw)*Delta*Omega_2* blkdiag(seesaw_R_w,seesaw_R_w)*As;
    
    JcMinv             = J/M;
    JcMinvSt           = JcMinv*St;
   
    pinv_JcMinvSt      = pinvDamped(JcMinvSt,reg.pinvDamp); 
    % null space
    nullJcMinvSt       = eye(robotDoFs) - pinv_JcMinvSt*JcMinvSt;
    
    % Mbar is the mass matrix associated with the joint dynamics, i.e.
    Mbar               = Mj-Mbj'/Mb*Mbj;
    NLMbar             = nullJcMinvSt*Mbar;
    
    % Adaptation of control gains for back compatibility with older
    % versions of the controller
    impedances         = gain.posturalProp*pinv(NLMbar,reg.pinvTol) + reg.impedances*eye(robotDoFs);
    dampings           = gain.posturalDamp*pinv(NLMbar,reg.pinvTol) + reg.dampings*eye(robotDoFs); 
    
    hjBar              = genBiasForces(7:end) - M(7:robotDoFs+6,1:6)/M(1:6,1:6)*genBiasForces(1:6) ...
                         -impedances* (qj-qjRef) -dampings * robotVel(7:end);

    JjBar              = transpose(J(:,7:end))-M(7:robotDoFs+6,1:6)/M(1:6,1:6)* transpose(J(:,1:6));

    tauModel           = LambdaPinv * (J / M * genBiasForces - Jdot_nu ...
                         + CONFIG.CONSIDER_SEESAW_DYN*blkdiag(w_R_seesaw, w_R_seesaw, w_R_seesaw, w_R_seesaw) *(Delta * Omega_1 + DeltaDot * s_omega_seesaw + ....
                                                    blkdiag(Sf(s_omega_seesaw),Sf(s_omega_seesaw),Sf(s_omega_seesaw),Sf(s_omega_seesaw)) * Delta * s_omega_seesaw)) + NullLambda*hjBar;

    Sigma              = -LambdaPinv*F - NullLambda*JjBar;
    SigmaNA            =  Sigma*NA; 

    f0                 = -pinvDamped(SigmaNA,reg.pinvDamp*1e-4)*(tauModel + Sigma*f_HDot);

    Sigmaf_HDot        = Sigma*f_HDot;

    fNoQp              = f_HDot  + NA*f0;
        
    %% QP parameters
    constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(w_R_lSole',w_R_lSole');
    constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(w_R_rSole',w_R_rSole');
    ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
    bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];

    ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
    bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*f_HDot;

    HessianMatrixQP2Feet      = SigmaNA'*SigmaNA + eye(size(SigmaNA,2))*reg.HessianQP;
    gradientQP2Feet           = SigmaNA'*(tauModel + Sigma*f_HDot);
    
end


