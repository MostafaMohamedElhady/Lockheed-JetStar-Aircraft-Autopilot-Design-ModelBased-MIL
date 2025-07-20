% This is the startup script that will run when we open the
% SIMULINK project at any time
% we will edit this code to take as input an excel file and assign
% variables

%% Importing data
filename_density_L = 'LH-JETSTAR.xlsx';
aircraft_data = xlsread(filename_density_L,'B2:B61');%% here B2:B61 means read the excel sheet from cell B2 to cell B61
%% Simulation settings
%Time vector parameters
T_Step = aircraft_data(1);
T_Final = aircraft_data(2);
lengths = T_Final/T_Step + 1;
Time_rvec = (0:T_Step:T_Final)';

%% Bus definitions
run States_Bus.m
%% Vehicle data
% gravity, mass % inertia
g = aircraft_data(52);
Ixx = aircraft_data(53);
Iyy = aircraft_data(54);
Izz = aircraft_data(55);
Ixz = aircraft_data(56);
Ixy=0;
Iyz=0;

Vehicle.Airframe.inertias = [Ixx , -Ixy , -Ixz ;...
                            -Ixy , Iyy , -Iyz ;...
                            -Ixz , -Iyz , Izz];
Vehicle.Airframe.mass = aircraft_data(51);

%% Initial conditions
initial.Vbody = aircraft_data(4:6);
initial.Vbody_dot = zeros(3,1);

initial.Vtot = sqrt(sum(initial.Vbody.^2));

initial.AngRates = aircraft_data(7:9);
initial.AngRates_dot = zeros(3,1);

initial.Euler_Angles = aircraft_data(10:12);
initial.Euler_Angles_dot = zeros(3,1);

initial.position = aircraft_data(13:15);
initial.position_dot = zeros(3,1);

% initial gravity force
initial.gravity = Vehicle.Airframe.mass*g*[sin(initial.Euler_Angles(2)); -cos(initial.Euler_Angles(2))*sin(initial.Euler_Angles(1)) ; -cos(initial.Euler_Angles(2))*cos(initial.Euler_Angles(1)) ];
initial.forces = initial.gravity;
% initial moments 
initial.moments = [0 0 0];

% Ref states
RefStates.x = initial.position(1);
RefStates.y = initial.position(2);
RefStates.z = initial.position(3);
RefStates.phi = initial.Euler_Angles(1);
RefStates.theta = initial.Euler_Angles(2);
RefStates.psi = initial.Euler_Angles(3);
RefStates.u = initial.Vbody(1);
RefStates.v = initial.Vbody(2);
RefStates.w = initial.Vbody(3);
RefStates.p = initial.AngRates(1);
RefStates.q = initial.AngRates(2);
RefStates.r = initial.AngRates(3);
RefStates.alpha = atan(RefStates.w./RefStates.u);
RefStates.beta = asin(RefStates.v./initial.Vtot);
RefStates.V_total = initial.Vtot;
RefStates.w_dot = 0;

%% Control Actions Values
% Aileron Rudder Elevator Thrust
controlInputs.DeltaElevator = aircraft_data(59)*pi/180;
controlInputs.DeltaAileron = aircraft_data(57)*pi/180;
controlInputs.DeltaRudder = aircraft_data(58)*pi/180;
controlInputs.DeltaThrust = aircraft_data(60);

%% Stability Derivatives
% Longitudinal motion
StabilityDerivatives_Longitudinal = aircraft_data(21:36);
StabilityDerivatives_Longitudinal = num2cell(StabilityDerivatives_Longitudinal);

[Xu, Zu, Mu, Xw, Zw, Mw, Zw_dot, Zq, Mw_dot, Mq, X_delta_elevator, Z_delta_elevator, M_delta_elevator, X_delta_thrust, Z_delta_thrust, M_delta_thrust] = deal(StabilityDerivatives_Longitudinal{:});

% Lateral motion
StabilityDerivatives_Lateral_dash = aircraft_data(37:50);

StabilityDerivatives_Lateral = Lateral_conv(StabilityDerivatives_Lateral_dash, Ixx, Izz, Ixz, initial.Vtot);
StabilityDerivatives_Lateral = num2cell(StabilityDerivatives_Lateral);

[Yv, YB, Y_delta_aileron, Y_delta_rudder, Lb_dash, Lp_dash, Lr_dash, Lda_dash, Ldr_dash, LB, Lp, Lr, L_delta_aileron, L_delta_rudder, NB_dash, Np_dash, Nr_dash, Nda_dash, Ndr_dash, NB, Np, Nr, N_delta_aileron, N_delta_rudder] = deal(StabilityDerivatives_Lateral{:});
Yp = 0;
Yr = 0;
Lv = LB/initial.Vtot;
Nv = NB/initial.Vtot;

Matrix_States = [Xu 0 Xw 0 0 0 0;
                0 Yv 0 Yp 0 Yr 0;
                Zu 0 Zw 0 Zq 0 Zw_dot;
                0 Lv 0 Lp 0 Lr 0;
                Mu 0 Mw 0 Mq 0 Mw_dot;
                0 Nv 0 Np 0 Nr 0];

Matrix_Controls = [0 X_delta_elevator X_delta_thrust 0
                   Y_delta_aileron 0 0 Y_delta_rudder
                   0 Z_delta_elevator Z_delta_thrust 0
                   L_delta_aileron 0 0 L_delta_rudder
                   0 M_delta_elevator M_delta_thrust 0
                   N_delta_aileron 0 0 N_delta_rudder];
