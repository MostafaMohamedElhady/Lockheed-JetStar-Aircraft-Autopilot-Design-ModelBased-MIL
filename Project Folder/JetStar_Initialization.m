clc
clearvars
% close all
%#ok<*NOPTS>

%% Running Start up needed codes
% Needed for Long, Lateral and Non-Linear
Start_File_Path = "MATLAB Models\Start Up\startup_script.m";
run(Start_File_Path)

%% Required TFs
servo = tf(10,[1 10]);
integrator = tf(1,[1 0]);
differentiator = tf([1 0],1);
engine_timelag = tf(0.1,[1 0.1]);

%% Running State Space Models needed codes + Controllers
SS_Long_File_Path = "MATLAB Models\State Space Models\SS_Longitudinal.m";
run(SS_Long_File_Path)
SS_Lateral_File_Path = "MATLAB Models\State Space Models\SS_Lateral.m";
run(SS_Lateral_File_Path)
%% Opening SIMULINK model
open_system('JetStar_MIL_Simulink.slx')



