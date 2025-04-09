clear all;
close all;
clc;
%%

T = 35.0;         % 총 시뮬레이션 시간
dt = 0.05;        % 시간 간격
N = round(T/dt); % 시뮬레이션 스텝 수

x = zeros(3, N);           % 상태 저장
u = zeros(2, N);           % 제어입력 저장

x_d_all = zeros(3, N);     % 목표 상태 궤적
xd_dot_all = zeros(3, N);  % 목표 상태 도함수

r = 3; w = 0.5;
for k = 1:N
  t = (k-1)*dt;
  x_d_all(:,k) = [r*cos(w*t); r*sin(w*t); w*t];
  xd_dot_all(:,k) = [-r*w*sin(w*t); r*w*cos(w*t); w];
  xd_dot_all(:,k) = [0;0;0];
end
for k = 1:N
  t = (k-1)*dt;
  x_desired = 0.01*t*t;
  y_desired = 0.13*t-0.04*t*t+0.001*t*t*t;
  x_d_all(:,k) = [x_desired; y_desired; atan2(y_desired, x_desired)];
  xd_dot_all(:,k) = [0;0;0];
end
#x(:,1) = [1.2*r; 0; pi/3];        % 초기 상태 (x, y, theta)
x(:,1) = [-3; 1.5; -pi/2];        % 초기 상태 (x, y, theta)

L = 0.3;
C = [1,0,0;
0,1,0;
0,0,0.1];

u_min = [-0.5; deg2rad(-90)];
u_max = [2.0; deg2rad(90)];
du_min = [-0.8; deg2rad(-45)];
du_max = [0.8; deg2rad(45)];

u_prev = [0.0; 0.0];

for k = 1:N-1
  x_k = x(:,k);
  x_d = x_d_all(:,k);
  xd_dot = xd_dot_all(:,k);

  % 초기 추정 입력
  if k == 1
    u0 = [0.0; 0];
  else
    u0 = u(:,k-1);
  end

  % SMC + QP 계산
  u_k = smc_qp(x_k, x_d, xd_dot, u0, u_prev, dt, L, C, u_min, u_max, du_min, du_max);

  % 상태 업데이트
  theta = x_k(3);
  v = u_k(1);
  delta = u_k(2);

  f = [
    v * cos(theta);
    v * sin(theta);
    v / L * tan(delta)
  ];

  x(:,k+1) = x_k + dt * f;
  u(:,k) = u_k;
  u_prev = u_k;
end

figure;
plot(x(1,:), x(2,:), 'b', 'LineWidth', 2); hold on;
plot(x_d_all(1,:), x_d_all(2,:), 'r--', 'LineWidth', 2);
legend('Actual Trajectory', 'Desired Trajectory');
xlabel('x'); ylabel('y'); grid on;
title('Sliding Mode Control with QP - Trajectory Tracking');

figure;
plot(x_d_all(1,:), x_d_all(2,:), 'k--', 'LineWidth', 2); hold on;
for i = 1:10:length(x)
  px = x(1,i);
  py = x(2,i);
  yaw = x(3,i);
  quiver(px,py,cos(yaw),sin(yaw),'b','linewidth',1);
end
plot(x(1,:), x(2,:), 'm', 'LineWidth', 1);
legend('Actual Trajectory', 'Desired Trajectory');
xlabel('x'); ylabel('y'); grid on;
title('Sliding Mode Control with QP - Trajectory Tracking');
axis equal;

figure();
plot(u(1,:)); hold on;
plot(u(2,:));