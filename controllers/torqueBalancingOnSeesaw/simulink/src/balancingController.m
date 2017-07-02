function [tauModel,SIGMA_fH,SIGMA_NA,ConstraintsMatrixQP2Feet,bVectorConstraintsQp2Feet,HessianMatrixQP2Feet,gradientQP2Feet,fH,NA,ddxCoM_seesaw] = ...
          balancingController(w_R_s,s_omega,seesaw,robot,trajectory,gain,qj,nu,ConstraintsMatrix,bVectorConstraints, ...
                              w_H_lSole,w_H_rSole,w_p_CoM,reg,CONFIG,ROBOT_DOF,xCoM_seesaw,dxCoM_seesaw)

%% Balancing control of iCub on a seesaw.                          
%% Recap of the equations needed for balancing on a semispherical seesaw.

% robot DoFs
nDof = size(ROBOT_DOF,1);

% data from the seesaw
delta = seesaw.delta;
rho = seesaw.rho;
mS = seesaw.mass;
Is = seesaw.inertia;
s_sr = seesaw.s_sr;
s_sl = seesaw.s_sl;

% distance between the CoM of the seesaw and the left/right foot position
% in world frame
w_sl = w_R_s * s_sl;
w_sr = w_R_s * s_sr;

% gravity acc
gravAcc = [0;0;-9.81];

% base for x-z 
e1 = [1;0;0];
e3 = [0;0;1];

% rotation between seesaw and world frame, w.r.t seesaw frame
s_R_w = transpose(w_R_s);

% angular velocity of the seesaw in world frame
%
%     w_omega = w_R_s * s_omega;

% distance between the contact with the ground and the seesaw CoM w.r.t
% world frame
w_r = (delta*w_R_s*e3) - rho*e3;

% velocity of the seesaw CoM in world frame
%
%     w_v = skew(w_r) * w_omega;

% matrix which projects the forces at feet into the seesaw centroidal
% dynamics, w.r.t the world frame
w_AS = [eye(3) zeros(3) eye(3) zeros(3);
        skew(w_sl) eye(3) skew(w_sr) eye(3)];

% The same matrix, projected in the seesaw frame
s_AS = [s_R_w zeros(3);
        zeros(3) s_R_w] * w_AS;
   
% also gravity needs to be proected in seesaw frame
s_g = s_R_w * gravAcc;

% distance w_r in seesaw frame
s_r = s_R_w * w_r;

% the seesaw velocity w.r.t. seesaw frame
%
%     s_v = skew(s_r) * s_omega;

% now, it is necessary to express the seesaw dynamics w.r.t. a frame
% attached to the seesaw, with it origin at the seesaw CoM

% inverse of seesaw inertia matrix
inv_Is = eye(3)/Is;

% linear momentum of the seesaw
%
%     s_vDot = skew(s_rDot)*s_omega +skew(s_r)*s_omegaDot = s_g -skew(s_omega)*s_v + (1/mS)*(s_Fc + s_Fe) (eq. 4)
%
% angular momentum  of the seesaw 
%
%     s_omegaDot = inv_Is*(skew(s_r)*s_Fc +s_Mue -skew(s_omega)*Is*s_omega) (eq. 5)

% substitute the seesaw angular momentum equation into the linear momentum
% equation (always w.r.t. seesaw frame) 
%
%     skew(s_rDot)*s_omega + skew(s_r)*inv_Is*(skew(s_r)*s_Fc + s_Mue - skew(s_omega)*Is*s_omega) =
%     s_g - skew(s_omega)*s_v + (1/mS)*(s_Fc + s_Fe)   (eq.6)

% multiplier of s_Fc in the equation that links the external force in seesaw 
% frame (s_Fc) to the forces and moments at feet in seesaw frame (s_Fe, s_Mue)
%
%     THETA_BAR = -(1/mS) * THETA;
%     IOTA = mS * inv_Is;
%     THETA = eye(3) + skew(s_r)*IOTA*transpose(skew(s_r));

% inverse of THETA
%
%     invTHETA = eye(size(THETA))/THETA;

% this equation links the external force in seesaw frame (s_Fc) to the
% forces and moments at feet in seesaw frame (s_Fe, s_Mue)  
%
%     s_Fc = invTHETA*(mS*skew(s_rDot)*s_omega +mS*skew(s_r)*inv_Is*(s_Mue - skew(s_omega)*Is*s_omega)-mS*s_g -s_Fe -mS*(skew(s_omega))^2*s_r); (eq. 7)
%

% substitute the seesaw contact force equation (7) into the seesaw angular
% momentum equation w.r.t. the seesaw frame (5)
%
%     Is*s_omegaDot = (eye(3)+mS*skew(s_r)*invTHETA*skew(s_r)*inv_Is)*s_Mue 
%                     -skew(s_r)*invTHETA*s_Fe -(eye(3)+mS*skew(s_r)*invTHETA*skew(s_r)*inv_Is -s_g)*skew(s_omega)*Is*s_omega 
%                     +ms*skew(s_r)*invTHETA*(skew(s_rDot)*s_omega-(skew(s_omega))^2*s_r);   (eq. 8)     
%

% now we need to find the force s_Fe and the torque s_Mue. We can use the
% feet acceleration equation in world frame
%
%     JDot*nu + J*nuDot = w_nufDot  (eq. 9)
%

% matrix that links the feet linear and angular velocity in seesaw frame to
% the angular velocity of the seesaw
DELTA = [skew(s_r-s_sl)
         eye(3)
         skew(s_r-s_sr)
         eye(3)];
 
% feet linear and angular velocity in seesaw frame
s_nuf = DELTA * s_omega;

% time derivative of s_r
s_rDot = rho*skew(s_omega)*s_R_w*e3;

% time derivative of DELTA
DELTA_dot = [eye(3)
             zeros(3)
             eye(3)
             zeros(3)]*skew(s_rDot);
         
% feet acceleration in seesaw frame
%
%     s_nufDot = DELTA_dot*s_omega + DELTA*s_omegaDot;

% matrix that projects the feet linear and angular velocity in world frame
w_Rbar_s = blkdiag(w_R_s, w_R_s, w_R_s, w_R_s);

% matrix used for computing the derivative of w_Rbar_s (w_RbarDot_s = w_Rbar_s * skewBar_omega)
skewBar_omega = blkdiag(skew(s_omega), skew(s_omega), skew(s_omega), skew(s_omega));

% feet linear and angular velocity in world frame
%
%     w_nuf = w_Rbar_s * s_nuf;  

% rewriting the equation that links the feet linear and angular velocity in 
% seesaw frame to the angular velocity of the seesaw (eq 9)
%
% JDot*nu + J*nuDot = w_Rbar_s*(skewBar_omega*s_nuf + DELTA_dot*s_omega + DELTA*s_omegaDot) (eq 12)

% rewriting the equation of the seesaw angular momentum w.r.t. the seesaw
% frame to have the following form: s_omegaDot = OMEGA1 + OMEGA2 * s_W_e
% in this way we can substitute s_omegaDot in (12)
%
%     OMEGA0 = IOTA + IOTA*skew(s_r)*invTHETA*skew(s_r)*IOTA;
%     OMEGA1 = IOTA*skew(s_r)*invTHETA*(skew(s_rDot)*s_omega-(skew(s_omega))^2*s_r -s_g) -OMEGA0*skew(s_omega)*Is*s_omega/mS;
%     OMEGA2 = [-IOTA*skew(s_r)*invTHETA, OMEGA0]/mS;

%% Extension for balancing on a semicylindrical seesaw: equations list

% in case of a semicylindrical seesaw, some of the equations below take a
% particular form

% constrain the rotation about z axis
%
%     s_Muc = transpose(w_R_s)*e3*Mu;

% add to s_r the component that create the torque along the y axis (it is
% constrained to be zero by the choice of eta)
%
%     s_rp = s_r + eta*e1

% angular momentum  of the seesaw: it contains now sMuc and s_rp instead of
% s_r (eq. 22)
%
%     s_omegaDot = inv_Is*(skew(s_rp)*s_Fc +s_Muc +s_Mue -skew(s_omega)*Is*s_omega)

% angular velocity of the seesaw has now only one element different from
% zero. It can be rewritten as: s_omega = thetaDot * e1
%
%     thetaDot = s_omega(1);

% then, eq 22 is simplified because in the x direction s_Muc is not
% effective. also, skew(s_omega)*s_omega is zero and
% transpose(e1)*skew(eta*e1) is zero too. hence, one has
%
% thetaDDot = transpose(e1)*inv_Is*(skew(s_r)*s_Fc +s_Mue)

% rewrite now the matrix IOTA (and THETA by consequence)
IOTA = e1*transpose(e1)*inv_Is*mS;
THETA = eye(3) + skew(s_r)*IOTA*transpose(skew(s_r));

% inverse of THETA
invTHETA = eye(size(THETA))/THETA;

% now we can find s_Fc exactly in the same way before
%
%     s_Fc = invTHETA*(mS*skew(s_rDot)*s_omega +skew(s_r)*IOTA*s_Mue -mS*s_g -s_Fe -mS*(skew(s_omega))^2*s_r); (eq. 29)
%

% NOW, we can proceed as before. substitute the seesaw contact force equation 
% into the seesaw angular momentum equation w.r.t. the seesaw frame
%
%     thetaDDot = transpose(e1)*inv_Is*(eye(3)+skew(s_r)*invTHETA*skew(s_r)*IOTA)*s_Mue
%                 + mS*transpose(e1)*inv_Is*skew(s_r)*invTHETA*(skew(s_rDot)*s_omega -(skew(s_omega))^2*s_r -s_g)
%                 - transpose(e1)*inv_Is*skew(s_r)*invTHETA*s_Fe

% feet acceleration equation in world frame
%
%     JDot*nu + J*nuDot = w_nufDot 
%
       
% feet acceleration in seesaw frame
%
%     s_nufDot = DELTA_dot*s_omega + DELTA*s_omegaDot;

% rewriting the equation that links the feet linear and angular velocity in 
% seesaw frame to the angular velocity of the seesaw
%
% JDot*nu + J*nuDot = w_Rbar_s*(skewBar_omega*s_nuf + DELTA_dot*s_omega + DELTA*e1*thetaDDot) (eq 12)

% rewriting the equation of the seesaw angular momentum w.r.t. the seesaw
% frame to have the following form: s_omegaDot = e1*thetaDDot = OMEGA1 + OMEGA2 * s_W_e
% in this way we can substitute s_omegaDot in (12)
OMEGA0 = IOTA + IOTA*skew(s_r)*invTHETA*skew(s_r)*IOTA;
OMEGA1 = IOTA*skew(s_r)*invTHETA*(skew(s_rDot)*s_omega-(skew(s_omega))^2*s_r -s_g);
OMEGA2 = [-IOTA*skew(s_r)*invTHETA, OMEGA0]/mS;
       
%% Use feet acceleration equation in world frame for linking the forces at 
%% feet with the control input, i.e. the joint torques

% add robot dynamics
M = robot.M;
J = robot.J;
h = robot.h;
H = robot.H;
intHw = robot.intHw;
JCoM = robot.JCoM;

w_p_lSole = w_H_lSole(1:3,4);
w_p_rSole = w_H_rSole(1:3,4);

w_R_lSole = w_H_lSole(1:3,1:3);
w_R_rSole = w_H_rSole(1:3,1:3);

JDot_nu = robot.JDot_nu;
qjDot = nu(7:end);

% add desired trajectories
qjDes = trajectory.qjDes;
xCoMDes = trajectory.desired_x_dx_ddx_CoM(:,1);
dxCoMDes = trajectory.desired_x_dx_ddx_CoM(:,2);
ddxCoMDes = trajectory.desired_x_dx_ddx_CoM(:,3);

% add gains matrices
impedances = gain.posturalProp;
dampings = gain.posturalDamp;
KthetaDot = gain.KthetaDot;
Ktheta = gain.Ktheta;

% projects the feet wrench in the seesaw frame
%
%     s_W_e = -s_AS * f

% substitute the robot dynamics in feet acceleration equation:
%
%     JDot*nu + J*invM*(S*tau + transpose(J)*f -h) = w_Rbar_s*(skewBar_omega*s_nuf + DELTA_dot*s_omega) + w_Rbar_s*DELTA*(OMEGA1 - OMEGA2*s_AS*f)

% calculate tau, and put in evidence the contact forces at feet:
%
%     tau = tauModel + SIGMA*fH + SIGMA*NA*f0 
%

% how to get tauModel: first, calculate hjBar:
hjBar = h(7:end) - M(7:nDof+6,1:6)/M(1:6,1:6)*h(1:6) -impedances*(qj-qjDes) -dampings*qjDot;
                     
% then, inverse of mass matrix, LAMBDA, its pseudoinverse and its null space
S = [zeros(6,nDof);
     eye(nDof,nDof)];
 
invM = eye(size(M))/M;

LAMBDA = J*invM*S;

pinvLAMBDA = pinvDamped(LAMBDA,reg.pinvDamp);

NLambda = eye(size(pinvLAMBDA * LAMBDA)) -pinvLAMBDA*LAMBDA;

% finally, it is possible to evaluate tauModel:
tauModel = pinvLAMBDA*(J*invM*h + w_Rbar_s*(skewBar_omega*s_nuf + DELTA_dot*s_omega + DELTA*OMEGA1) -JDot_nu) + NLambda*hjBar;

% how to get SIGMA: first, calculate F and JjBar:
F = J*invM*transpose(J) + w_Rbar_s*DELTA*OMEGA2*s_AS;

JjBar = transpose(J(:,7:end)) -M(7:nDof+6,1:6)/M(1:6,1:6)*transpose(J(:,1:6));

% finally, SIGMA:
SIGMA = -pinvLAMBDA*F -NLambda*JjBar;

%% CONTROL part: it is in charge of retrieve the forces fH and their null space NA

% compatibility with other controllers
ddxCoM_seesaw = zeros(3,1);
    
if CONFIG.CONTROL_KIND == 1
    
    % CONTROL 1: fH is the vector of desired feet wrenches. In this
    % configuration, it is in charge of stabilizing the robot momentum only
   
    % distance between the robot CoM and the feet
    w_gl = w_p_lSole - w_p_CoM;
    w_gr = w_p_rSole - w_p_CoM;
    
    % matrix which projects the forces at feet into the robot centroidal
    % dynamics, w.r.t the world frame and its pseudoinvers
    w_AR = [eye(3) zeros(3) eye(3) zeros(3)
            skew(w_gl) eye(3) skew(w_gr) eye(3)];
        
    pinvAR = pinv(w_AR, reg.pinvTol);
   
    % linear velocity of the robot CoM 
    w_vCoM = JCoM * nu;
    w_vCoM = w_vCoM(1:3);
    
    % gravity wrench in world frame   
    gravityWrench = [M(1,1)*gravAcc;zeros(3,1)];
    
    % Null space of w_AR
    NA = eye(size(pinvAR*w_AR)) - pinvAR*w_AR;
  
    % saturate the CoM position error
    saturated_xCoM = saturate(gain.PCOM * (xCoMDes - w_p_CoM(1:3)),-gain.P_SATURATION,gain.P_SATURATION);
  
    % desired CoM acceleration
    dotH_seesaw = ddxCoMDes + saturated_xCoM + gain.DCOM * (dxCoMDes - w_vCoM);

    % robot desired linear and angular momentum
    HDot_star = [ M(1,1)*dotH_seesaw; 
                 -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw];
             
    % desired forces at feet
    fH =  pinvAR*(HDot_star - gravityWrench);
     
elseif CONFIG.CONTROL_KIND == 2
    
    % CONTROL 2: fH is the vector of desired feet wrenches. In this
    % configuration, it is in charge of stabilizing the robot momentum only
    % but the CoM reference is such that the angular momentum of the seesaw
    % is stabilized
    
    % distance between the robot CoM and the feet
    w_gl = w_p_lSole - w_p_CoM;
    w_gr = w_p_rSole - w_p_CoM;
    
    % first, external forces multipliers and bias forces acting on
    % seesaw ang. mom. dynamics
    momentMultipl    =  transpose(e1)*(eye(3)+skew(s_r)*invTHETA*skew(s_r)*IOTA);
    forceMultipl     = -transpose(e1)*skew(s_r)*invTHETA;
    biasForcesSeesaw =  mS*transpose(e1)*skew(s_r)*invTHETA*(skew(s_rDot)*s_omega -(skew(s_omega))^2*s_r -s_g);
    
    % then, combine forces and moments and project them to the world frame
    Aseesaw          = -[forceMultipl,momentMultipl]*s_AS;
    
    % desired thetaDDot 
    thetaDot         = s_omega(1);
    seesaw_angles    = rollPitchYawFromRotation(w_R_s);
    thetaDDot_star   = -KthetaDot*thetaDot -Ktheta*seesaw_angles(1);
    
    % forces at feet that generates desired thetaDDot:
    %    pinvAseesaw = pinv(Aseesaw,reg.pinvTol);
    %    NAseesaw    = eye(size(pinvAseesaw*Aseesaw)) - pinvAseesaw*Aseesaw;
     
    % matrix which projects the forces at feet into the robot centroidal
    % dynamics, w.r.t the world frame and its pseudoinverse
    w_AR = [eye(3) zeros(3) eye(3) zeros(3)
            skew(w_gl) eye(3) skew(w_gr) eye(3)];
        
    pinvAR = pinv(w_AR, reg.pinvTol);
    
    % evaluate the desired robot momentum derivative that stabilizes the angular 
    % momentum of the seesaw in the x direction 
    A_dotH      = Aseesaw*pinvAR;
    pinvA_dotH  = pinv(A_dotH,reg.pinvTol);    
    dotH_seesaw = pinvA_dotH*(Is(1)*thetaDDot_star -biasForcesSeesaw);
   
    % linear velocity of the robot CoM 
    w_vCoM = JCoM * nu;
    w_vCoM = w_vCoM(1:3);
    
    % gravity wrench in world frame   
    gravityWrench = [M(1,1)*gravAcc;zeros(3,1)];
    
    % Null space of w_AR
    NA = eye(size(pinvAR*w_AR)) - pinvAR*w_AR;
  
    % desired CoM acceleration from desired momentum derivative
    ddxCoM_seesaw = dotH_seesaw(1:3)/M(1,1);
    
    % saturate the CoM position error
    saturated_xCoM = saturate(gain.PCOM * (xCoM_seesaw - w_p_CoM(1:3)),-gain.P_SATURATION,gain.P_SATURATION);

    % desired CoM acceleration
    ddxCoM_star = saturated_xCoM + gain.DCOM * (dxCoM_seesaw - w_vCoM);

    % robot desired linear and angular momentum
    HDot_star_feedback = [ M(1,1)*ddxCoM_star; 
                          -gain.DAngularMomentum*H(4:end)-gain.PAngularMomentum*intHw];
       
    HDot_star = dotH_seesaw + HDot_star_feedback;       
             
    % desired forces at feet
    fH =  pinvAR*(HDot_star - gravityWrench);
 
else
    
    fH = zeros(12,1);
    NA = zeros(12,12);
    
end
                          
%% Optimization using quadratic programming (QP) solver

% parameters from control
SIGMA_NA = SIGMA * NA; 
SIGMA_fH = SIGMA * fH;

% QP parameters
constraintMatrixLeftFoot  = ConstraintsMatrix * blkdiag(transpose(w_R_lSole),transpose(w_R_lSole));
constraintMatrixRightFoot = ConstraintsMatrix * blkdiag(transpose(w_R_rSole),transpose(w_R_rSole));
ConstraintsMatrix2Feet    = blkdiag(constraintMatrixLeftFoot,constraintMatrixRightFoot);
bVectorConstraints2Feet   = [bVectorConstraints;bVectorConstraints];

ConstraintsMatrixQP2Feet  = ConstraintsMatrix2Feet*NA;
bVectorConstraintsQp2Feet = bVectorConstraints2Feet-ConstraintsMatrix2Feet*fH;
HessianMatrixQP2Feet      = transpose(SIGMA_NA)*SIGMA_NA + eye(size(SIGMA_NA,2))*reg.HessianQP;
gradientQP2Feet           = transpose(SIGMA_NA)*(tauModel + SIGMA*fH);

end





