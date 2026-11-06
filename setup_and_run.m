clearvars; close all; clc;
addpath('general/', 'uav/', 'payload/', 'payload_control/',  'trajectory/')

%% Rigid cable parameters and numerical stabilization
cable = struct;
% Both cables have the same rigid length.
cable.length = [1; 1];

% Numerical stabilization gains for the holonomic constraint:
%
% phi_ddot + 2*zeta*wn*phi_dot + wn^2*phi = 0
%
% in order to suppress numerical drift from ||x_ai - x_i|| = d_i during integration.
cable.wn = 20.0;      % [rad/s]
cable.zeta = 1.0;

%% System description
% General UAV
uav = struct;
uav.mass = 0.4;
uav.inertia_matrix = diag([0.0021 0.0021 0.0041]);
uav.initial_position = [0 0 0]';
uav.initial_linear_velocity = [0 0 0]';
uav.initial_angular_velocity = [0 0 0]';
uav.initial_quaternion = [1 0 0 0]';

%% Payload description
% Parameters
payload = struct;

payload.mass = 0.225;
% Uniform rectangular bar geometry.
payload.length = 1.00;      % [m], aligned with x_P
payload.width  = 0.08;      % [m], along y_P
payload.height = 0.06;      % [m], along z_P

Lb = payload.length;
wb = payload.width;
hb = payload.height;
mp = payload.mass;

payload.inertia_matrix = diag([ ...
    mp * (wb^2 + hb^2) / 12, ...
    mp * (Lb^2 + hb^2) / 12, ...
    mp * (Lb^2 + wb^2) / 12 ]);
% About UAVs 
payload.number_uav = 2;
payload.Rattach = [ ...
     Lb/2, -Lb/2;
     0.0,   0.0;
     0.0,   0.0 ];
% Initial conditions
payload.initial_position = [0 0 0]';
payload.initial_linear_velocity = [0 0 0]';
payload.initial_angular_velocity = [0 0 0]';
payload.initial_quaternion = [1 0 0 0]';

%% Initial position of UAVs
uav1 = uav;
uav2 = uav;
clear uav;

uav1.initial_position = [ Lb/2;
                           0.0;
                          -cable.length(1) ];

uav2.initial_position = [-Lb/2;
                           0.0;
                          -cable.length(2) ];

%% Initialize buses
initBus_quadrotor_state;
initBus_payload_state;
initBus_payload_reference;

%% General control tuning 
% Position control
pos_ctrl = struct;
pos_ctrl.Kd = 2;
pos_ctrl.alpha = 6; % 6 % 2
pos_ctrl.mass = 0.4;
% Attitude control
att_ctrl = struct;
att_ctrl.Kd = 1;
att_ctrl.beta = 0.3;
att_ctrl.gamma = 10;
att_ctrl.rho = 18;
% Payload control (nominal controller)
payload_ctrl = struct;
payload_ctrl.mass = payload.mass;
payload_ctrl.Ks(1:3,1:3) = diag( [ 0, 0, 1] );
payload_ctrl.Ks(4:6,4:6) = diag( [ 0, 0.25, 0 ] );
payload_ctrl.Lambda_pos = diag( [ 0, 0, 2 ] );
payload_ctrl.Lambda_att = diag( [ 0, 2.0, 0 ] );


%% Run simulator and see visualization
out = sim("simulator_maats_wrench.slx");
visualizer( out.logsout, 'Rattach', payload.Rattach, 'ShowTrajectories', true, 'DroneSize', 100, 'FrameStride', 20);
