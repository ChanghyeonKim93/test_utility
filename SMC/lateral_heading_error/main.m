clc; clear all; close all;

%% Load parameters
load_parameters;

parameters.control.gain_lateral_error = 10.0;
parameters.control.gain_heading_error = 5.0;

use_short_wheelbase   = true;
use_tip_lateral_error = true;

parameters.base_to_tip = 0.2;

sim_time_scaler = 10;
if(use_short_wheelbase) 
  parameters.steer_to_rotation_center = 0.5;
endif

% Destination and stopby point
X_stopby = [-0.0;0.0;0.0];
X_goal   = [-4.0; 0.0; 0.0]; 

X = [2.0; -1.0; -1.4];
%X = [2.0; 1.5; pi/2+0.1];
U = [0.0; 0.0];
U_delay = zeros(2, floor(parameters.motion.delay / dt)+1);

V_baselink = [0;0;0];

simulation_time = 0.0;
t = [0.0];
X_des = X_goal;
while(1)
  if(t(end) > 60.0)
    break;
  end

  Xk = X(:, end);

  if(abs(Xk(1) - X_goal(1)) < 0.1 && abs(Xk(2) - X_goal(2)) < 0.02)
    break;
  end

  direction_sign = -1;
  Uk=[0;0];
  if(use_tip_lateral_error)
    Uk = lateral_heading_error_controller_tip(Xk, U_delay(:,1), X_des, parameters, direction_sign);
  else
    Uk = lateral_heading_error_controller(Xk, U_delay(:,1), X_des, parameters, direction_sign);
  endif
  U_delay = [U_delay, Uk];
  U_sim = U_delay(:,1);
  U_delay(:,1)=[];
  L = parameters.steer_to_rotation_center;
  Xdot = [
    U_sim(1)*cos(U_sim(2))*cos(Xk(3)); 
    U_sim(1)*cos(U_sim(2))*sin(Xk(3)); 
    U_sim(1)/L*sin(U_sim(2))
  ];
  
  X = [X, Xk + Xdot*dt];
  U = [U, Uk];
  
  [vx,vy,w] = ComputeBaselinkVelocities(Xk,Xdot,parameters);
  V_baselink = [V_baselink,[vx;vy;w]];

  simulation_time = simulation_time + dt;
  t = [t,simulation_time];
end

%%
% Drawing
figure(1);
hold on;
plot(t, U(1,:), 'b', 'linewidth',2);
plot(t, U(2,:), 'r', 'linewidth',2);
xlabel('Time [sec]'); ylabel('Control input');
title('Control input');
grid on;
legend('Linear velocity [m/s]', 'Steering angle [rad]');

figure(2);
plot(X(1,:), X(2,:), 'b', 'linewidth', 2); hold on;
plot([X_stopby(1),X_goal(1)], [X_stopby(2),X_goal(2)], 'r--', 'linewidth',2);
legend('Real trajectory', 'Desired trajectory');
xlabel('x'); ylabel('y');
grid on; axis equal;
title('SMC trajectory');

figure('position',[10,10,1080,540]);
footprint = parameters.footprint;
subplot(1,2,1);
footprint_sides=[footprint.length/2, -footprint.length/2, -footprint.length/2, footprint.length/2;
  footprint.width/2,footprint.width/2, -footprint.width/2, -footprint.width/2];
rotation_center_vector=[0;0.1];

T_Rc_C = GetPose(-(parameters.base_to_steer_length-parameters.footprint.offset.x-parameters.steer_to_rotation_center), 0, 0);
T_Rc_Sc = GetPose(parameters.steer_to_rotation_center, 0, 0);
T_C_tip = GetPose(-parameters.footprint.length/2, 0, 0);

h_rc_traj  = plot([0,0],[0,0],'linewidth',1,'b..'); hold on;
h_tip_traj = plot([0,0],[0,0],'linewidth',1,'m..'); 
h_fs = plot([0,0],[0,0],'linewidth',2,'k');
h_bs = plot([0,0],[0,0],'linewidth',2,'k');
h_ls = plot([0,0],[0,0],'linewidth',2,'k');
h_rs = plot([0,0],[0,0],'linewidth',2,'k');
h_rc_x = plot([0,0],[0,0],'linewidth',4,'r');
h_steer = plot([0,0],[0,0],'linewidth',6,'k');
h_left_wheel = plot([0,0],[0,0],'linewidth',6,'k');
h_right_wheel = plot([0,0],[0,0],'linewidth',6,'k');

plot([X_stopby(1),X_goal(1)], [X_stopby(2),X_goal(2)], 'r--', 'linewidth',2);
grid minor; axis equal;
xlim([-4,4]); ylim([-4,4]);

subplot(2,2,2);
h_w = plot([0,0],[0,0],'r','linewidth',2); hold on;
h_v = plot([0,0],[0,0],'b','linewidth',2);
legend({'steer angle','steer wheel velocity'});
xlim([0, t(end)]); grid minor;

subplot(2,2,4);
h_vx = plot([0,0],[0,0],'r','linewidth',2); hold on;
h_vy = plot([0,0],[0,0],'g','linewidth',2); 
h_wb = plot([0,0],[0,0],'b','linewidth',2);
legend({'vx','vy','wb'});
xlim([0, t(end)]); grid minor;

X_tip = [0;0];
r_wheel = 0.15;

for k = 1:sim_time_scaler:length(X)
  Xk = X(:,k);
  Uk = U(:,k);
  
  % Rotation center 
  T_G_Rc = GetPose(Xk(1),Xk(2),Xk(3));
  
  % Footprint
  T_G_C = T_G_Rc*T_Rc_C;
  S = T_G_C(1:2,1:2)*footprint_sides + T_G_C(1:2,3);
  
  T_G_Rce = T_G_Rc * GetPose(0.2, 0, 0);
  
  % Steer center
  T_G_Sc = T_G_Rc * T_Rc_Sc;
  
  % Steer wheel f/b
  T_G_Sf = T_G_Sc * GetPose(0,0,Uk(2)) * GetPose( r_wheel, 0, 0);
  T_G_Sb = T_G_Sc * GetPose(0,0,Uk(2)) * GetPose(-r_wheel, 0, 0);
  
  % Passive roller
  track_width = 0.5;
  T_G_LWc = T_G_Rc * GetPose(0, -track_width/2, 0);
  T_G_LWf = T_G_LWc * GetPose(r_wheel, 0, 0);
  T_G_LWb = T_G_LWc * GetPose(-r_wheel, 0, 0);
  T_G_RWc = T_G_Rc * GetPose(0,  track_width/2, 0);
  T_G_RWf = T_G_RWc * GetPose(r_wheel, 0, 0);
  T_G_RWb = T_G_RWc * GetPose(-r_wheel, 0, 0);
  
  T_G_tip = T_G_C * T_C_tip;
  
  X_tip = [X_tip, [T_G_tip(1,3);T_G_tip(2,3)]];
  
  set(h_rc_traj, 'XData', X(1,1:k), 'YData', X(2,1:k));
  set(h_tip_traj, 'XData', X_tip(1,1:end), 'YData', X_tip(2,1:end));
  
  set(h_fs, 'XData', S(1,1:2), 'YData', S(2,1:2));
  set(h_ls, 'XData', S(1,2:3), 'YData', S(2,2:3));
  set(h_bs, 'XData', S(1,3:4), 'YData', S(2,3:4));
  set(h_rs, 'XData', [S(1,4),S(1,1)], 'YData', [S(2,4),S(2,1)]);
  set(h_rc_x, 'XData', [T_G_Rc(1,3), T_G_Rce(1,3)], 'YData', [T_G_Rc(2,3), T_G_Rce(2,3)]);
  set(h_steer, 'XData', [T_G_Sf(1,3), T_G_Sb(1,3)], 'YData', [T_G_Sf(2,3), T_G_Sb(2,3)]);
  set(h_right_wheel, 'XData', [T_G_RWf(1,3), T_G_RWb(1,3)], 'YData', [T_G_RWf(2,3), T_G_RWb(2,3)]);
  set(h_left_wheel, 'XData', [T_G_LWf(1,3), T_G_LWb(1,3)], 'YData', [T_G_LWf(2,3), T_G_LWb(2,3)]);
  
  set(h_v, 'XData', t(1:k), 'YData', U(1,1:k));
  set(h_w, 'XData', t(1:k), 'YData', U(2,1:k));
  
  set(h_vx, 'XData', t(1:k), 'YData', V_baselink(1,1:k));
  set(h_vy, 'XData', t(1:k), 'YData', V_baselink(2,1:k));
  set(h_wb, 'XData', t(1:k), 'YData', V_baselink(3,1:k));
  
  drawnow;
  pause(dt);
endfor
