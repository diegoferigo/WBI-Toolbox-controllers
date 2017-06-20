function [ ss_position, ss_vel, robot_pos, robot_vel ] = state_partitioning(state, robotDofs)

% STATE_PARTITIONING 

% vectors dimensions
seesaw_pos_len           = 4;
seesaw_vel_len           = 3;
robot_pos_len            = 7 + robotDofs;
robot_vel_len            = 6 + robotDofs;

seesaw_pos_initial_index = 1;
robot_pos_initial_index  = seesaw_pos_initial_index + seesaw_pos_len;
seesaw_vel_initial_index = robot_pos_initial_index  + robot_pos_len;
robot_vel_initial_index  = seesaw_vel_initial_index + seesaw_vel_len;

ss_position              = state(seesaw_pos_initial_index:seesaw_pos_initial_index + seesaw_pos_len - 1);
robot_pos                = state(robot_pos_initial_index : robot_pos_initial_index + robot_pos_len - 1);

ss_vel                   = state(seesaw_vel_initial_index : seesaw_vel_initial_index + seesaw_vel_len - 1);
robot_vel                = state(robot_vel_initial_index : robot_vel_initial_index + robot_vel_len - 1);

end
