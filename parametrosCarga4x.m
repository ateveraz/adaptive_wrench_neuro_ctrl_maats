clearvars; close all; clc;
addpath('general/', 'uav/', 'payload/')

%% General control tuning 
% Position control
pos_ctrl = struct;
pos_ctrl.Kd = 2;
pos_ctrl.alpha = 6;
pos_ctrl.mass = 0.4;
% Attitude control
att_ctrl = struct;
att_ctrl.Kd = 1;
att_ctrl.beta = 0.3;
att_ctrl.gamma = 10;
att_ctrl.rho = 18;

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


%% ===========================
% Rigid cable parameters
%% ===========================

cable = struct;

% Both cables have the same imposed rigid length.
cable.length = [1; 1];

% Numerical stabilization gains for the holonomic constraint:
%
% phi_ddot + 2*zeta*wn*phi_dot + wn^2*phi = 0
%
% These values do not model elasticity. They suppress numerical
% drift from ||x_ai - x_i|| = d_i during integration.
cable.wn = 20.0;      % [rad/s]
cable.zeta = 1.0;

% Initial position of UAVs
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

%% Maybe I will remove it frome here ! 
% Datos Dinámica
% 
% mL = 0.225;
% e3 = [0;0;1];
% mi = 0.4;
% J = diag([2.1,1.87,3.97])*0.01;
% %% Box information
% Lx = 0.4;   % largo 
% Ly = 0.3;   % ancho 
% Lz = 0.2;   % alto 
% 
% r1 = [ Lx/2;  Ly/2;  Lz/2];
% r2 = [ Lx/2; -Ly/2;  Lz/2];
% r3 = [-Lx/2;  Ly/2;  Lz/2];
% r4 = [-Lx/2; -Ly/2;  Lz/2];
% 
% 
% %% Calcular las qis
% L1=1;
% L2=1;
% L3=1;
% L4=1;
% 
% m = 0.225;
% g = 9.81*e3;
% 
% %% Initial conditions
% xL0=[0.8;0.8;0.1];
% q1_rot=rotation_matrix(-25, 'z')*rotation_matrix(-35,'y');
% q2_rot=rotation_matrix(65, 'z')*rotation_matrix(-35,'y');
% q3_rot=rotation_matrix(155, 'z')*rotation_matrix(-35,'y');
% q4_rot=rotation_matrix(245, 'z')*rotation_matrix(-35,'y');
% 
% x1_0=xL0+q1_rot*[0;0;L1];
% x2_0=xL0+q2_rot*[0;0;L2];
% x3_0=xL0+q3_rot*[0;0;L3];
% x4_0=xL0+q4_rot*[0;0;L4];
% 
% d1 = (xL0-x1_0)/L1;
% d2 = (xL0-x2_0)/L2; 
% d3 = (xL0-x3_0)/L3;
% d4 = (xL0-x4_0)/L4;
% 
% n1 = norm(d1); 
% n2 = norm(d2); 
% n3 = norm(d3);
% n4 = norm(d4);
% 
% q1_0 = d1/n1;
% q2_0 = d2/n2;
% q3_0 = d3/n3;
% q4_0 = d4/n4;
% 
% %%
% Kd1 = 2; Alpha1 = 6; Gamma1 = 0;
% % Kd2 = 5; Alpha2 = 1; Gamma2 = 0;
% % Kd3 = 5; Alpha3 = 1; Gamma3 = 0;
% % Kd4 = 5; Alpha4 = 1; Gamma4 = 0;
% 
% %sim('simulator.slx');

%% Init buses
% initBus_quadrotor_state;
% initBus_payload_state;