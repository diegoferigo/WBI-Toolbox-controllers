function w_R_seesaw = seesawRotationInWorldframe(imu_H_lSole,imu_H_lSole_0,imuRotation_0,imuRotation,neckPosition,CONFIG)

% Transformation between the world and l_sole at time 0
w_H_lSole_0 = CONFIG.w_H_lSole_0;

% Transformation between world and robot imu at time 0
w_H_imu_0   = w_H_lSole_0/imu_H_lSole_0;
w_R_imu_0   = w_H_imu_0(1:3,1:3);

% Converting the IMU angles from grad into rad
imuRot_rad        = (imuRotation * pi)/180;
imuRot_rad_0      = (imuRotation_0 * pi)/180;

% Composing the rotation matrix:
% See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12
wImu_R_imu      = rotz(imuRot_rad(3))*roty(imuRot_rad(2))*rotx(imuRot_rad(1));
wImu_R_imu_0    = rotz(imuRot_rad_0(3))*roty(imuRot_rad_0(2))*rotx(imuRot_rad_0(1));

% Relative rotation between wImu and the world (should be constant!)
w_R_wImu        = w_R_imu_0/wImu_R_imu_0;

% Correcting neck movements
wImu_H_wImuAssumingNeckToZero = correctIMU(neckPosition);
wImu_R_imu                    = wImu_H_wImuAssumingNeckToZero(1:3,1:3) * wImu_R_imu;

% Rotation of lSole (= seesaw) w.r.t. the imu world
imu_R_lSole  = imu_H_lSole(1:3,1:3);
wImu_R_lSole = wImu_R_imu * imu_R_lSole;

% Filter the pitch and yaw to be zero
rollPitchYaw_lSole = rollPitchYawFromRotation(wImu_R_lSole);

if CONFIG.YAW_IMU_FILTER
    rollPitchYaw_lSole(3) = 0;
end
if CONFIG.PITCH_IMU_FILTER
    rollPitchYaw_lSole(2) = 0;
end

wImu_R_lSole = rotz(rollPitchYaw_lSole(3))*roty(rollPitchYaw_lSole(2))*rotx(rollPitchYaw_lSole(1));

% Finally, rotation of the seesaw w.r.t the world
w_R_seesaw   = w_R_wImu * wImu_R_lSole;

end
