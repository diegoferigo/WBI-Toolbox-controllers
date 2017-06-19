function w_data = rotateImuWorldFrame(wImu_data)

% the IMU mounted on the seesaw has its reference frame (wImu) oriented
% differently from the world reference frame (w). Therefore, this function
% project the acceleration/velocity of the seesaw from the wImu to the world
% frame.
w_R_wImu  = rotx(pi) * rotz(-44/180*pi);

w_data = w_R_wImu * wImu_data;

end