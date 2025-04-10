clc;
clear all;
close all;

%% Parameters
dt = 0.02;        % 시간 간격
params.L = 0.7;
params.u_min = [0.0; -1.4];
params.u_max = [0.5;  1.4];
params.du_max = [0.3*dt; 0.8*dt];
params.delay = 0.0; % [sec]

params.lambda_ey = 5.0;
params.lambda_et = 4.0;

params.kd = 10.5;
params.epsilon = 70.0;

vel_max = 1.2; % [m/s]
steer_max = pi/2; % [rad]

X_goal = [4.0; 0.1; 0.0];
X_predock = [1.0;0.1; 0.0];

X_init = [-1.0; 0.0; -0.0];

X = X_init;
u = [0;0];
uc = zeros(2, floor(params.delay/dt)+1);

time = 0.0;7
k = 1;
X_desired = X_predock;
is_predock_reached = false;

t = [time];
while(1)
  if(time > 200.0)
    break;
  end
  
  Xk = X(:,k);
  if(~is_predock_reached && abs(Xk(1)-X_desired(1)) < 0.5 && abs(Xk(2)-X_desired(2)) < 0.5)
    X_desired = X_goal;
   
    params.lambda_ey = 4.0;
    params.lambda_et = 8.0;
    is_predock_reached = true;
  end
  if(is_predock_reached && abs(Xk(1)-X_desired(1)) < 0.01 && abs(Xk(3)-X_desired(3)) < 0.01)
    break;
  end

  uk = smc_bicycle_controller(Xk, uc(:,1), X_desired, params);
  uc(:,1) = [];
  uc = [uc, uk];
  f = [
    uc(1,1) * cos(Xk(3));
    uc(1,1) * sin(Xk(3));
    uc(1,1) / params.L * sin(uc(2,1));
  ];

  X = [X, Xk + dt * f];
  u = [u, uk];
  
  k = k+1;
  time += dt;
  t=[t,time];
end

figure;
plot(t,u(1,:), 'b', 'LineWidth', 2); hold on;
plot(t,u(2,:), 'r', 'LineWidth', 2); 
xlabel('time [sec]'); ylabel('y'); 
grid on;
axis equal;

figure;
plot(X(1,:), X(2,:), 'b', 'LineWidth', 2); hold on;
plot([X_predock(1),X_goal(1)],[X_predock(2),X_goal(2)], 'r--', 'LineWidth', 2);
legend('Actual Trajectory', 'Desired Trajectory');
xlabel('x'); ylabel('y'); 
grid on;
axis equal;
title('Sliding Mode Control with QP - Trajectory Tracking');

footprint_length = 2.0;
footprint_width = 0.8;
wheelbase = 0.3;

figure();
side_lines=[footprint_length/2+wheelbase, -footprint_length/2+wheelbase, -footprint_length/2+wheelbase, footprint_length/2+wheelbase;
 footprint_width/2,footprint_width/2,-footprint_width/2,-footprint_width/2];

h_fs = plot([0,0],[0,0],'linewidth',2); hold on;
h_bs = plot([0,0],[0,0],'linewidth',2);
h_ls = plot([0,0],[0,0],'linewidth',2);
h_rs = plot([0,0],[0,0],'linewidth',2);
plot([X_predock(1),X_goal(1)],[X_predock(2),X_goal(2)], 'r--', 'LineWidth', 2);
grid minor;
axis equal;
xlim([-2,14]); ylim([-8,8]);
for k = 1:5:length(X)
  Xk = X(:,k);
  
  R = [cos(Xk(3)),-sin(Xk(3));sin(Xk(3)),cos(Xk(3))];
  S = Xk(1:2,1)+R*side_lines;
  set(h_fs,'XData', S(1,1:2), 'YData', S(2,1:2));
  set(h_ls,'XData', S(1,2:3), 'YData', S(2,2:3));
  set(h_bs,'XData', S(1,3:4), 'YData', S(2,3:4));
  set(h_rs,'XData', [S(1,1),S(1,4)], 'YData', [S(2,1),S(2,4)]);
  
  drawnow;
  pause(0.03);
endfor


% figure;
% plot(x_d_all(1,:), x_d_all(2,:), 'k--', 'LineWidth', 2); hold on;
% for i = 1:10:length(x)
%   px = x(1,i);
%   py = x(2,i);
%   yaw = x(3,i);
%   quiver(px,py,cos(yaw),sin(yaw),'b','linewidth',1);
% end
% plot(x(1,:), x(2,:), 'm', 'LineWidth', 1);
% legend('Actual Trajectory', 'Desired Trajectory');
% xlabel('x'); ylabel('y'); grid on;
% title('Sliding Mode Control with QP - Trajectory Tracking');
% axis equal;

% figure();
% plot(u(1,:)); hold on;
% plot(u(2,:));