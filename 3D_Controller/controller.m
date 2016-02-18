function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

phi_des = 0;
theta_des = 0;
Kdz = 15;
Kpz = 20;


F = params.mass*(params.gravity + des_state.acc(3) + Kdz*(des_state.vel(3)-state.vel(3)) + Kpz*(des_state.pos(3)-state.pos(3)));

Kdx = 1;
Kpx = 15;

Kdy = .01;
Kpy = 45;

Kpp = .7;
Kdp = .001;

Kpt = 10;
Kdt = 0;

Kppsi = 10;
Kdpsi = 0;

r1_des = des_state.acc(1) + Kdx*(des_state.vel(1)-state.vel(1)) + Kpx*(des_state.pos(1)-state.pos(1));
r2_des = des_state.acc(2) + Kdy*(des_state.vel(2)-state.vel(2)) + Kpy*(des_state.pos(2)-state.pos(2));

phi_des   = 1/params.gravity*(r1_des*des_state.yaw-r2_des);
theta_des = 1/params.gravity*(r1_des+r2_des*des_state.yaw);

Phi   = Kpp*(phi_des-state.rot(1))   + Kdp*(0-state.omega(1));

Theta = Kpt*(theta_des-state.rot(2)) + Kdt*(0-state.omega(2));

Psi   = Kppsi*(des_state.yaw-state.rot(3)) + Kdpsi*(des_state.yawdot-state.omega(3));

M = [Phi; Theta; Psi];


end
