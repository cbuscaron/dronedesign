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


% =================== Your code goes here ===================

%Kvz = 10;
%Kpz = 1000;

%Kvp = 100;
%Kpp = 3000;

%Kvy = 40;
%Kpy = 39.5;
phi_des = 0;
theta_des = 0;
Kdz = 15;
Kpz = 20;


F = params.mass*(params.gravity + des_state.acc(3) + Kdz*(des_state.vel(3)-state.vel(3)) + Kpz*(des_state.pos(3)-state.pos(3)));

Kdx = 1;
Kpx = 10;

Kdy = 1;
Kpy = 30;

Kpp = .5;
Kdp = 0.1;

Kpt = 10;
Kdt = 0.1;

Kppsi = 10;
Kdpsi = 0.1;

r1_des = des_state.acc(1) + Kdx*(des_state.vel(1)-state.vel(1)) + Kpx*(des_state.pos(1)-state.pos(1));
r2_des = des_state.acc(2) + Kdy*(des_state.vel(2)-state.vel(2)) + Kpy*(des_state.pos(2)-state.pos(2));

phi_des   = 1/params.gravity*(r1_des*des_state.yaw-r2_des);
theta_des = 1/params.gravity*(r1_des+r2_des*des_state.yaw);

Phi   = Kpp*(phi_des-state.rot(1))   + Kdp*(0-state.omega(1));

Theta = Kpt*(theta_des-state.rot(2)) + Kdt*(0-state.omega(2));

Psi   = Kppsi*(des_state.yaw-state.rot(3)) + Kdpsi*(des_state.yawdot-state.omega(3));

M = [Phi; Theta; Psi];


%Pc =(-1/params.gravity)*(Kvy*(-des_state.vel(1)) + Kpy*(des_state.pos(1) - state.pos(1)));
%Pc = (-1/params.gravity)*(des_state.acc(1) + Kvy*(des_state.vel(1)- state.vel(1)) + Kpy*(des_state.pos(1) - state.pos(1)));
 
%F = params.mass*(params.gravity - Kvz*state.vel(2) + Kpz*(des_state.pos(2) - state.pos(2)));
%M = params.I*(Kvp*(0 - state.omega(1)) + Kpp*(Pc - state.rot(1)));

%des   = [des_state.pos(1); des_state.pos(2)];
%state_is = [state.pos(1); state.pos(2); state.vel(1); state.vel(2); state.rot(1); state.omega(1)];
% Thurst
%F = 0;

% Moment
%M = zeros(3,1);

% =================== Your code ends here ===================

end
