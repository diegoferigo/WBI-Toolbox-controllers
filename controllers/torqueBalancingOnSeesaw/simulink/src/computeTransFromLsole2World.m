function w_H_lSole = computeTransFromLsole2World(w_R_seesaw, seesaw)
    
    % Angles associated with w_R_s
    seesaw_rpy = rollPitchYawFromRotation(w_R_seesaw);
 
    % s_p_lSole = positionOfLeftFoot - seesawCoM  w.r.t. seesaw frame
    s_p_lSole  = [seesaw.lFootDistance_x;  seesaw.lFootDistance_y; seesaw.top];

    % OC = centerOfRotationSeesaw - originOfWorld  w.r.t world frame
    OC         = [ 0;
                  -seesaw.rho * seesaw_rpy(1);
                   seesaw.rho];

    % CL = positionOfLeftFoot - centerOfRotationSeesaw w.r.t. world frame
    e3        = [ 0; 0; 1 ];
    CL        = w_R_seesaw * (s_p_lSole - seesaw.delta*e3);   

    % P_l = positionOfLeftFoot w.r.t. world frame
    P_l       = OC + CL;
    
    w_H_lSole = [w_R_seesaw,   P_l;
                 zeros(1,3),   1];
              
end
