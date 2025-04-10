function u = smc_bicycle_controller(X, u_prev, X_des, params)
  delay = params.delay;
  
  % Estimate delayed state
  vel_prev = u_prev(1); steer_prev = u_prev(2);
  x_est   = X(1) + vel_prev * cos(X(3)) * delay;
  y_est   = X(2) + vel_prev * sin(X(3)) * delay;
  psi_est = X(3) + vel_prev / params.L * sin(steer_prev) * delay;
 
  dpsi = vel_prev / params.L * sin(steer_prev);
 
  % 오차 계산
  goal_direction = [cos(X_des(3)); sin(X_des(3))];
  X_est = [x_est;y_est];
  ep = X_est(1:2,1) - X_des(1:2,1);
  ey = [-goal_direction(2),goal_direction(1)] * ep;
  et = X_des(3) - psi_est;
  if(et > pi) 
    et = et - 2*pi;
  end
  
  % Sliding surface
  s = -params.lambda_ey * ey + params.lambda_et * et;
    
  ep_measure = X_des(1:2,1)-X(1:2,1);
  ex = ep_measure.'*goal_direction;
  
  % 5. 슬라이딩 제어
  u_smc = [0.0;  params.kd * tanh(s / params.epsilon)];
  u_eq = [min(abs(0.3*ex(1)*cos(et)*cos(abs(dpsi))), params.u_max(1)); 0];

  u = u_eq + u_smc;
  
  % 6. 입력 saturate
  dv_max = params.du_max(1);
  dw_max = params.du_max(2);
  if(u(1) < u_prev(1)-1.5*dv_max)
    u(1) = u_prev(1)-1.5*dv_max;
  elseif(u(1) > u_prev(1)+dv_max)
    u(1) = u_prev(1)+dv_max;
  end
  if(u(2) < u_prev(2)-dw_max)
    u(2) = u_prev(2)-dw_max;
  elseif(u(2) > u_prev(2)+dw_max)
    u(2) = u_prev(2)+dw_max;
  end
  u(1) = max(min(u(1), params.u_max(1)), params.u_min(1));
  u(2) = max(min(u(2), params.u_max(2)), params.u_min(2));
end
