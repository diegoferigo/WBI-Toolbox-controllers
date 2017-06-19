function w_v_base  = computeBaseVelocity(J_lSole, w_R_seesaw,dq_j, w_omega_seesaw, model)

    w_omega_s      = w_R_seesaw*w_omega_seesaw;

    e3             = [0;0;1];
    
    w_r            = model.seesaw.delta * w_R_seesaw * e3 - model.seesaw.rho * e3 ; 

    s_s_l          = [ 0; 
                       model.seesaw.lFootDistanceCenter; 
                       model.seesaw.top];

    w_s_l          = w_R_seesaw * s_s_l;

    w_v_l_sole     = [Sf(w_r - w_s_l)*w_omega_s; 
                                      w_omega_s  ];

    w_v_base       = J_lSole(1:6,1:6)\(w_v_l_sole - J_lSole(1:6,7:end)*dq_j);
    
end