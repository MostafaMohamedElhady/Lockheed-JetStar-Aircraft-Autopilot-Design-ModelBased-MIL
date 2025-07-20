%% State Space Definition Function
function State_sapce_model = State_Space(A, B)
    % Define the state-space system
    C = eye(length(A)); % C = Identity matrix to output all states
    D = zeros(size(B));  % D = zero for no feedthrough
    State_sapce_model = ss(A, B, C, D);
end

%% Initial Parameters
mass = Vehicle.Airframe.mass;

% Euler angles
phi0   = initial.Euler_Angles(1);
theta0 = initial.Euler_Angles(2);
psi0   = initial.Euler_Angles(3);

% Velocities
Vt0 = initial.Vtot;
u0 = initial.Vbody(1);
v0 = initial.Vbody(2);
w0 = initial.Vbody(3);

%% Lateral Full Dynamics
% Lateral Dynamics (beta, p, r, phi, psi)
A_lat = [Yv      (w0+Yp)/Vt0 (Yr-u0)/Vt0   g*cos(theta0)/Vt0  0;
         Lb_dash Lp_dash     Lr_dash       0                  0;
         NB_dash Np_dash     Nr_dash       0                  0;
         0       1           tan(theta0)   0                  0;
         0       0           1/cos(theta0) 0                  0];

B_lat = [Y_delta_aileron      Y_delta_rudder;
         Lda_dash             Ldr_dash;
         Nda_dash             Ndr_dash;
         0                    0;
         0                    0];

C_lat = eye(5);
D_lat = zeros(5,2);

lateral_full_model = State_Space(A_lat, B_lat);
lateral_tf_full = minreal(tf(lateral_full_model));

%% Transfer Functions: Aileron Response
tf_full_beta_da = zpk(lateral_tf_full(1,1));
tf_full_p_da = zpk(lateral_tf_full(2,1));
tf_full_r_da = zpk(lateral_tf_full(3,1));
tf_full_phi_da = zpk(lateral_tf_full(4,1));
tf_full_psi_da = zpk(lateral_tf_full(5,1));

%% Transfer Functions: Rudder Response
tf_full_beta_dr = zpk(lateral_tf_full(1,2));
tf_full_p_dr = minreal((lateral_tf_full(2,2)));
tf_full_r_dr = zpk(lateral_tf_full(3,2));
tf_full_phi_dr = zpk(lateral_tf_full(4,2));
tf_full_psi_dr = zpk(lateral_tf_full(5,2));

%% Yaw Damper Control 
OL_r_rcom = servo*tf_full_r_dr; % OpenLoop
Yaw_Damper = load("Lateral SISO controllers\Yaw_Damper_New_Struct.mat");
HPF_YawDamper = Yaw_Damper.C*0.5961/2.5376; % should be the one corresponding to the following controllers

%% Roll Controller
Roll_controller_SISO = load("Lateral SISO controllers\Roll_controller_damped.mat");
Roll_controller_C1 = Roll_controller_SISO.Roll_C1;
Roll_controller_C2 = Roll_controller_SISO.Roll_C2;
%% Control System Parameters
% Coordinated turn time constant
tau = 6.5;

% Aileron limits
Aileron_pos_limit_deg = 50;  % max change in aileron 
Aileron_neg_limit_deg = -50; % min change in aileron 

% Rudder limits
Rudder_pos_limit_deg = 15;   % max change in rudder 
Rudder_neg_limit_deg = -15;  % min change in rudder 

% Bank angle limits
phi_pos_limit_deg = 30;      % max change in Bank angle 
phi_neg_limit_deg = -30;     % min change in Bank angle 
