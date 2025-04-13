function Uk = lateral_heading_error_controller2(X, U_prev, X_des, parameters, direction_sign)
  L = parameters.steer_to_rotation_center;
  gain_lateral = parameters.control.gain_lateral_error;
  gain_heading = parameters.control.gain_heading_error;
  v_max = parameters.motion.steer.max_linear_vel;

  % Estimate delayed state
  vel_prev   = U_prev(1); 
  steer_prev = U_prev(2);

  % Error calculation
  gd = [cos(X_des(3)); sin(X_des(3))]; % goal direction 
  gd_skew = [-gd(2);gd(1)];
  
  error_position = X_des(1:2,1)-X(1:2,1);
  error_lateral  = gd_skew.' * error_position;
  error_heading  = direction_sign*(X_des(3) - X(3));
  if(error_heading > pi) 
    error_heading = error_heading - 2*pi;
  end

  % Sliding surface
  s = gain_lateral * error_lateral + gain_heading * error_heading;

  error_position_measure = X_des(1:2,1)-X(1:2,1);
  error_x = direction_sign * error_position_measure.' * gd;

  % Sliding control
  v_brake = sqrt(abs(error_x));
  if(abs(error_x) < 0.2) 
    v_brake = 0.01*direction_sign*error_x;
  end

  error_lateral_regulator = min(0.3, abs(cos(error_lateral)));
  error_heading_regulator = min(0.3, abs(cos(error_heading)));

  v_des = direction_sign * min(v_brake, v_max);
  
  xdot = v_des*cos(X(3));
  ydot = v_des*sin(X(3));
  ey_dot = gd_skew.'*[xdot;ydot] ;
  sin_steer_des = (-gain_lateral*ey_dot
            + parameters.motion.steer.max_steer_angle * tanh(s * parameters.control.steer_effort * 0.1))/(gain_heading*v_des/L);
  if(sin_steer_des > 0.99) 
    sin_steer_des = 0.99;
  elseif(sin_steer_des < -0.99)
    sin_steer_des = -0.99;
  end
  steer_des = asin(-sin_steer_des);

  Uk_eq = [v_des; 0.0];
  Uk_smc = [0.0; steer_des];

  Uk = Uk_eq + Uk_smc;
  
  % Compute s dot
  psidot = v_des/L*sin(steer_des);

  sdot = -gain_lateral*[-gd(2),gd(1)] * [xdot;ydot] - gain_heading*psidot;
  if(s*sdot>0.0)
    s*sdot
  end
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