function [ F, M ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

Kvz = 10;
Kpz = 1000;

Kvp = 100;
Kpp = 3000;

Kvy = 40;
Kpy = 39.5;



%Pc =(-1/params.gravity)*(Kvy*(-des_state.vel(1)) + Kpy*(des_state.pos(1) - state.pos(1)));
Pc = (-1/params.gravity)*(des_state.acc(1) + Kvy*(des_state.vel(1)- state.vel(1)) + Kpy*(des_state.pos(1) - state.pos(1)));
 
F = params.mass*(params.gravity - Kvz*state.vel(2) + Kpz*(des_state.pos(2) - state.pos(2)));
M = params.Ixx*(Kvp*(0 - state.omega(1)) + Kpp*(Pc - state.rot(1)));

des   = [des_state.pos(1); des_state.pos(2)];
state_is = [state.pos(1); state.pos(2); state.vel(1); state.vel(2); state.rot(1); state.omega(1)];
end

