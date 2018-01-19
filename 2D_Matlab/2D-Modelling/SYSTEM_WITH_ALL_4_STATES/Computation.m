%% Clean-up Workspace
clc;
clear all;

%% Load Model Parameter
Parameter_flag = 3; 
%syms m_K m_W m_A r_K r_W r_A l Theta_K Theta_W Theta_Wxy Theta_A Theta_Axy phi_x phi_x_dot theta_x theta_x_dot T_x g i 
syms phi_z phi_z_dot phi_x phi_x_dot phi_y phi_y_dot theta_z theta_z_dot theta_x theta_x_dot theta_y theta_y_dot T_x T_y T_z T_f 

if Parameter_flag == 1
    Parameters_Zuerich
elseif Parameter_flag == 2
    Parameters_Group
else
    Parameter_Geschaetzt
end


%% Compute Model
Systemdesign_YZ;
%Systemdesign_XZ;
%Systemdesign_XY;

% Arbeitspunkte
x_AP = [0;0;0;0];
T_x_AP=0;
T_y_AP=0;
T_z_AP=0;

% Anfangsbedingungen
x_0_yz =[0 0 0 0];
x_0_xz =[0 0 0 0];
x_0_xy =[0 0 0 0];

