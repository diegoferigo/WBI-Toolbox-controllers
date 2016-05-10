function [w_H_root,wImu_R_link] = fromBaseToWorldWithImu(imu_H_link,imu_H_link_0,link_H_root,inertial_0,inertial,CONFIG)
%#codegen

% Converting the inertial values from grad into rad
inertial     = (inertial   * pi)/180;
inertial_0   = (inertial_0 * pi)/180;

% Composing the rotation matrix:
% See http://wiki.icub.org/images/8/82/XsensMtx.pdf page 12
wImu_R_imu      = rotz(inertial(3))*roty(inertial(2))*rotx(inertial(1));
wImu_R_imu_0    = rotz(inertial_0(3))*roty(inertial_0(2))*rotx(inertial_0(1));

imu_R_link   = imu_H_link(1:3,1:3);
imu_R_link_0 = imu_H_link_0(1:3,1:3);


wImu_R_link     = wImu_R_imu*imu_R_link;
wImu_R_link_0   = wImu_R_imu_0*imu_R_link_0;

if CONFIG.YAW_IMU_FILTER
    rollPitchYaw_link_0 = rollPitchYawFromRotation(wImu_R_link_0);
    rollPitchYaw_link   = rollPitchYawFromRotation(wImu_R_link);
    wImu_R_link         = rotz(rollPitchYaw_link_0(3))*roty(rollPitchYaw_link(2))*rotx(rollPitchYaw_link(1));
end

wImu_H_link     = [wImu_R_link,zeros(3,1)
                    zeros(1,3),1];
          
wImu_H_link_0   = [wImu_R_link_0,zeros(3,1)
                    zeros(1,3),1];

wImu_H_root     = wImu_H_link*link_H_root;

w_H_root        = wImu_H_link_0\wImu_H_root;
