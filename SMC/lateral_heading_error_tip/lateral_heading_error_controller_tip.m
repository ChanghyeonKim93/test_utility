function Uk = lateral_heading_error_controller_tip(X, U_prev, X_des, parameters, direction_sign)
  L = parameters.steer_to_rotation_center;
  gain_lateral = parameters.control.gain_lateral_error;
  gain_heading = parameters.control.gain_heading_error;
  v_max = parameters.motion.steer.max_linear_vel;
  braking_distance = parameters.braking_distance;

  % Estimate delayed state
  vel_prev   = U_prev(1); 
  steer_prev = U_prev(2);

  % Error calculation
  g = [cos(X_des(3)); sin(X_des(3))]; % goal direction 
  g_normal = [-g(2);g(1)];

  B = parameters.base_to_steer_length - parameters.steer_to_rotation_center;

  X_tip = X(1:2,1) - B*[cos(X(3)); sin(X(3))];
  
  error_position = X_des(1:2,1)-X_tip(1:2,1);
  error_lateral  = g_normal.' * (X_des(1:2,1)-X(1:2,1));
  error_lateral_tip  = g_normal.' * (X_des(1:2,1)-X_tip(1:2,1));

  % Sliding surface
  s = gain_lateral * error_lateral + gain_heading * error_lateral_tip;

  error_position_measure = X_des(1:2,1)-X(1:2,1);
  error_x = direction_sign *  g.' * error_position_measure;

  % Sliding control
  v_brake = sqrt(abs(error_x));
  
  if(abs(error_x) < braking_distance ) 
    v_brake = 0.1*abs(error_x)/braking_distance;
  end

  v_des = direction_sign * min(v_brake, v_max);
  
  X_dot = v_des*cos(U_prev(2))*[cos(X(3));sin(X(3))];
  X_tip_dot = X_dot + v_des*B/L*sin(U_prev(2))*[sin(X(3));-cos(X(3))];

  ey_dot = g_normal.'*X_tip_dot;
  
  %ey_dot = Clamp(ey_dot, -0.5,0.5);
  sin_steer_des = (-gain_lateral*ey_dot
            + tanh(s * parameters.control.steer_effort * 0.1))/(gain_heading*(direction_sign*v_max)/L);
  sin_steer_des = Clamp(sin_steer_des, -0.98, 0.98);
  steer_des = direction_sign*asin(sin_steer_des);

  Uk_eq = [v_des; 0.0];
  Uk_smc = [0.0; steer_des];

  Uk = Uk_eq + Uk_smc;
  
  % Compute s dot
  %psidot = v_des/L*sin(steer_des);
  %sdot = -gain_lateral*[-gd(2),gd(1)] * [xdot;ydot] - gain_heading*psidot;
  %if(s*sdot>0.0)
  %  s*sdot
  %end
 % s*sdot

  % Input saturation
  dt = 0.02;
  max_acc = parameters.motion.steer.max_acceleration*dt;
  max_dec = 1.5 * parameters.motion.steer.max_acceleration*dt;
  dw_max  = parameters.motion.steer.max_steer_angle_rate*dt;
  if(Uk(1) < U_prev(1)-max_dec)
    Uk(1) = U_prev(1)-max_dec;
  elseif(Uk(1) > U_prev(1)+max_acc)
    Uk(1) = U_prev(1)+max_acc;
  end
  if(Uk(2) < U_prev(2)-dw_max)
    Uk(2) = U_prev(2)-dw_max;
  elseif(Uk(2) > U_prev(2)+dw_max)
    Uk(2) = U_prev(2)+dw_max;
  end
  Uk(1) = max(min(Uk(1), parameters.motion.steer.max_linear_vel), -parameters.motion.steer.max_linear_vel);
  Uk(2) = max(min(Uk(2), parameters.motion.steer.max_steer_angle), -parameters.motion.steer.max_steer_angle);
end