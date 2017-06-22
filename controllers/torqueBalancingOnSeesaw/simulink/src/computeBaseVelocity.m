function w_v_base  = computeBaseVelocity(J_lSole, w_R_seesaw, dq_j, s_omega_seesaw, model)

    % conversion to the velocity of the seesaw expressed in world frame
    w_omega_s      = w_R_seesaw * s_omega_seesaw;

    % should be the distance between the seesaw CoM and the contact point
    % of the seesaw with the ground
    e3             = [0;0;1];
    w_r            = model.seesaw.delta * w_R_seesaw * e3 - model.seesaw.rho * e3 ; 

    % s_p_lSole = positionOfLeftFoot - seesawCoM  w.r.t. seesaw frame
    s_p_lSole      = [model.seesaw.lFootDistance_x;  model.seesaw.lFootDistance_y; model.seesaw.top];

    % positionOfLeftFoot - seesawCoM  w.r.t. world frame
    w_p_lSole      = w_R_seesaw * s_p_lSole;

    % linear and angular velocity of lSole in the world coordinates
    w_v_lSole     = [Sf(w_r - w_p_lSole)*w_omega_s; 
                                         w_omega_s];
                                     
    % finally, base velocity in world coodinates
    invJb         = pinv(J_lSole(1:6,1:6), model.reg.pinvDampVb);
    w_v_base      = invJb*(w_v_lSole - J_lSole(1:6,7:end)*dq_j);
    
end