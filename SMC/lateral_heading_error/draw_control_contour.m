clc; clear all; close all;

%% Load parameters
load_parameters;

parameters.control.gain_lateral_error = 5.0;
parameters.control.gain_heading_error = 2.0;

% Destination and stopby point
X_stopby = [-2.0; 0.0; 0.0];
X_goal   = [-4.0; 0.0; 0.0]; 
X_des = X_goal;

direction_sign = -1;

%% Make convergence map
position_step = 0.5;
yaw_sim = -0.6;

half_len = 20;
len = half_len*2;

X_map = zeros(2*half_len,2*half_len);
Y_map = zeros(2*half_len,2*half_len);

dx_map = zeros(2*half_len,2*half_len);
dy_map = zeros(2*half_len,2*half_len);

parameters.motion.steer.max_acceleration = 1000;
parameters.motion.steer.max_steer_angle_rate = 1000;
dt = 0.2;
for i = -half_len+1:half_len
  x = position_step * i;
  for j = -half_len+1:half_len
    y = position_step * j;
    X = [x;y;yaw_sim];
    U = [0;0];
    for iter = 1:10
      U  = lateral_heading_error_controller(X, U, X_des, parameters, direction_sign);
      dX = ComputeRotationCenterVelocity(X, U, parameters.steer_to_rotation_center);
      X = X+dX*dt;
    endfor
    
    X_map(i+half_len,j+half_len) = x;
    Y_map(i+half_len,j+half_len) = y;
    dx_map(i+half_len,j+half_len) = [X(1)-x];
    dy_map(i+half_len,j+half_len) = [X(2)-y];
    %dx_map(i+100,j+100) = cos(u(2));
    %dy_map(i+100,j+100) = sin(u(2));
  endfor
endfor

figure();
quiver(X_map,Y_map,dx_map,dy_map);
grid on;
axis equal;
