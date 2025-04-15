function Uk = lateral_heading_error_controller_tip(X, U_prev, X_des, parameters, direction_sign)
  L = parameters.steer_to_rotation_center;
  B = parameters.base_to_steer_length-parameters.steer_to_rotation_center+parameters.base_to_tip; % rotation center to tip
  
  gain_lateral = parameters.control.gain_lateral_error;
  gain_heading = parameters.control.gain_heading_error;
  v_max = parameters.motion.steer.max_linear_vel;

  % Estimate delayed state
  vel_prev   = U_prev(1); 
  steer_prev = U_prev(2);

  % Error calculation
  gd = [cos(X_des(3)); sin(X_des(3))]; % goal direction 
  gd_skew = [-gd(2);gd(1)];
  
  X_tip = X(1:2,1) - B*[cos(X(3));sin(X(3))];
  
  error_lateral  = gd_skew.' * (X_des(1:2,1)-X_tip(1:2,1));
  error_heading  = direction_sign*(X_des(3)-X(3));
  if(error_heading > pi) 
    error_heading = error_heading - 2*pi;
  end

  error_position_measure = X_des(1:2,1)-X(1:2,1);
  error_x = direction_sign * gd.' * error_position_measure;

  % Velocity control
  v_prevent_overshoot = v_max;
  v_brake       = v_max * abs(error_x) / 0.2;;
  v_min_angular = v_max * cos(Clamp(error_heading,-0.5,0.5)) * cos(Clamp(U_prev(2),-1.3,1.3));
  
  v_desired     = direction_sign * min([v_brake, v_max, v_min_angular]);
  
  % Sliding surface
  s = gain_lateral * error_lateral + gain_heading * error_heading;

  xdot = U_prev(1)*cos(U_prev(2))*cos(X(3));
  ydot = U_prev(1)*cos(U_prev(2))*sin(X(3));
  psidot = U_prev(1)/L*tan(U_prev(2));
  xtip_dot = xdot +B*sin(X(3))*psidot;
  ytip_dot = ydot -B*cos(X(3))*psidot;
  ey_dot = gd_skew.'*[xtip_dot;ytip_dot] ;
  sin_steer_des = (Clamp(-gain_lateral*ey_dot, -0.3, 0.3)+
            + tanh(s * parameters.control.steer_effort * 0.1))/(gain_heading*direction_sign*v_max/L);

  sin_steer_des = Clamp(sin_steer_des, -0.999, 0.999);
  steer_des = asin(direction_sign*sin_steer_des);

  Uk_eq = [v_desired/cos(steer_des); 0.0];
  Uk_smc = [0.0; steer_des];

  Uk = Uk_eq + Uk_smc;
  
  % Compute s dot

  sdot = -gain_lateral*[-gd(2),gd(1)] * [xtip_dot;ytip_dot] - gain_heading*psidot;


  % Input saturation
  delay_scaler = (1+parameters.motion.delay)*(1+parameters.motion.delay);
  dt = 0.02;
  max_acc = parameters.motion.steer.max_acceleration*dt*delay_scaler;
  max_dec = 1.5 * parameters.motion.steer.max_acceleration*dt*delay_scaler;
  dw_max  = parameters.motion.steer.max_steer_angle_rate*dt*delay_scaler;
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