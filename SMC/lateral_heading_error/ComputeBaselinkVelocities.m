function [vx,vy,w] = ComputeBaselinkVelocities(X,Xdot,params)
B = params.base_to_steer_length - params.steer_to_rotation_center;  
vx = Xdot(1) + B*sin(X(3))*Xdot(3);
vy = Xdot(2) - B*cos(X(3))*Xdot(3);
w  = Xdot(3);
endfunction
