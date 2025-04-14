% Simulation parameters
dt = 0.0125; % [s] // 80Hz

% Robot footprint
parameters.footprint.length = 1.8;
parameters.footprint.width = 0.8;
parameters.footprint.offset.x = 0.5;
parameters.footprint.offset.y = 0.0;

parameters.base_to_steer_length = 1.2;
parameters.steer_to_rotation_center = 0.5;

parameters.motion.steer.max_linear_vel = 0.3;
parameters.motion.steer.max_acceleration = 0.3;
parameters.motion.steer.max_steer_angle = deg2rad(88); % rad
parameters.motion.steer.max_steer_angle_rate = deg2rad(45); % rad/s

parameters.motion.delay = 0.0;

parameters.control.gain_lateral_error = 7.0;
parameters.control.gain_heading_error = 2.0;

parameters.control.scaler_linear_vel = 1.0;
parameters.control.scaler_steer = 10.0;

parameters.control.steer_effort = 3.0; % 0~1

parameters.braking_distance = 0.4;