function u_opt = smc_qp(x, x_d, xd_dot, u0, u_prev, dt, L, C, u_min, u_max, du_min, du_max)
  theta = x(3);
  v = u0(1);
  delta = u0(2);

  % f(x, u0)
  f0 = [
    v * cos(theta);
    v * sin(theta);
    v / L * tan(delta)
  ];

  % ∂f/∂u
  df_du = [
    cos(theta),         -v * sin(theta);
    sin(theta),          v * cos(theta);
    tan(delta)/L,        v / L * (1 / cos(delta)^2)
  ];

  % A and b
  A = C * df_du;   % (3x2)
  b = C * (x - x_d) + 2*C * (f0 - xd_dot);   % (3x1)

  % QP terms
  H = 2 * (A' * A);
  f_qp = 2 * (A' * b);

  % Constraints on u
  lb = u_min;
  ub = u_max;

  % Constraints on u_dot
  lb_dot = du_min;
  ub_dot = du_max;

  lb = max(lb, u_prev + dt * lb_dot);
  ub = min(ub, u_prev + dt * ub_dot);

  % QP call
  x0 = u0;
  Aeq = []; beq = [];

  [u_opt, obj, info, lambda] = qp(x0, H, f_qp, Aeq, beq, lb, ub);

end