function Xdot = ComputeRotationCenterVelocity(X, U, L)
v_wheel = U(1);
steer_angle = U(2);

Xdot = [0;0;0];
Xdot(1) = v_wheel*cos(steer_angle)*cos(X(3));
Xdot(2) = v_wheel*cos(steer_angle)*sin(X(3));
Xdot(3) = v_wheel/L*sin(steer_angle);
  
endfunction
