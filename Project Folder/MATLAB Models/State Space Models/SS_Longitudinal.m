%% State Space Definition
function State_sapce_model = State_Space(A,B)
% Define the state-space system
C = eye(length(A)); % C = Identity matrix to output all states
D = zeros(size(B)); % D = zero for no feedthrough
State_sapce_model = ss(A,B,C,D);
end
mass  =  Vehicle.Airframe.mass;

%% Long Full Dynamics
A_Full_Linear_Long = [...
    Xu , Xw, -initial.Vbody(3), -g*cos(initial.Euler_Angles(2));
    Zu/(1 - Zw_dot), Zw/(1 - Zw_dot), (Zq + initial.Vbody(1))/(1 - Zw_dot), -g*sin(initial.Euler_Angles(2))/(1 - Zw_dot);
    (Mu + Mw_dot*Zu/(1 - Zw_dot)), (Mw + Mw_dot*Zw/(1 - Zw_dot)), (Mq + Mw_dot*(Zq + initial.Vbody(1))/(1 - Zw_dot)), -Mw_dot*g*sin(initial.Euler_Angles(2))/(1 - Zw_dot);
    0 0 1 0];

B_Full_Linear_Long = [...
    X_delta_elevator, X_delta_thrust;
    Z_delta_elevator/(1 - Zw_dot), Z_delta_thrust/(1 - Zw_dot);
    (M_delta_elevator + Mw_dot*Z_delta_elevator/(1 - Zw_dot)), (M_delta_thrust + Mw_dot*Z_delta_thrust/(1 - Zw_dot));
    0, 0];
C_Full_Linear_Long = eye(4);
D_Full_Linear_Long = zeros(4,2);

Long_full_model =  State_Space(A_Full_Linear_Long,B_Full_Linear_Long);
tf_full_long = minreal(tf(Long_full_model));

%% TFs: Due to Elevator
tf_full_u_de = zpk(tf_full_long(1,1));
tf_full_w_de = zpk(tf_full_long(2,1));
tf_full_q_de = zpk(tf_full_long(3,1));
tf_full_theta_de = zpk(tf_full_long(4,1));
alpha_de = tf_full_w_de/RefStates.u;
gamma_de = tf_full_theta_de-alpha_de;
wdot_de = tf_full_w_de*differentiator;
az_de = wdot_de - RefStates.u*tf_full_q_de;

%% TFs: Due to Thrust
tf_full_u_dth = zpk(tf_full_long(1,2));
tf_full_w_dth = zpk(tf_full_long(2,2));
tf_full_q_dth = zpk(tf_full_long(3,2));
tf_full_theta_dth = zpk(tf_full_long(4,2));
alpha_dT = tf_full_w_dth/RefStates.u;

%% Elevator limits
Elevator_pos_limit_deg = 25;
Elevator_neg_limit_deg = -25;

%% Throttle control action limits (for saturation block in SIMULINK)
% Drag estimation (Thrust available for control action = Engines total thrust - Drag)
S_Jetstar = 542.5; % ft^2
CD_Jetstar = 0.095;
Q_Jetstar = 78.4; % psf
Drag_Jetstar = S_Jetstar * CD_Jetstar * Q_Jetstar; % pound-force
AvailableThrust_Jetstar = 4 * 3700; % pound-force, 4 engines 
AvailableThrust_Jetstar = AvailableThrust_Jetstar - Drag_Jetstar;

Throttle_pos_limit_lbf = AvailableThrust_Jetstar; % max change in throttle lever (already we are compensating for drag)
Throttle_neg_limit_lbf = -(Drag_Jetstar - 3700); % min change in throttle lever (reaching quarter of the thrust from four engines - reaching 3700 lbf)

%% Preparing SISO designed controllers for SIMULINK 
% Pitch controller
C1_pitch = load("Longitudinal SISO controllers\PID_C1.mat");
C2_pitch = load("Longitudinal SISO controllers\PID_C2.mat");
kp_pitch = C1_pitch.C1_Design3.K;
ki_pitch = kp_pitch;
kd_pitch = C2_pitch.C2_Design3.K;
PI_pitch = tf([kp_pitch ki_pitch],[1,0]);

% Velocity controller
C1_vel = load("Longitudinal SISO controllers\C1_vel.mat");
C2_vel = load("Longitudinal SISO controllers\C2_vel.mat");

kp_vel = C1_vel.C1_vel.K;
ki_vel = kp_vel;
PI_Velocity = tf([kp_vel ki_vel],[1 0]);

Lead_gain = C2_vel.C2_vel.K;
Lead_Velocity = tf([Lead_gain Lead_gain*0.1],[1 1]);

% Altitude controller
kp_alt = 1.0542e-04;
ki_alt = 1.0542e-04;
kd_alt = 0.0012;

PI_Altitude = tf([kp_alt ki_alt],[1 0]);
