clc;
clear all;

Parameter_flag = 0; 
%syms m_K m_W m_A r_K r_W r_A l Theta_K Theta_W Theta_Wxy Theta_A Theta_Axy phi_x phi_x_dot theta_x theta_x_dot T_x g i 
syms phi_z phi_z_dot phi_x phi_x_dot theta_z theta_z_dot theta_x theta_x_dot T_x T_y T_z T_f 

if Parameter_flag == 0
    Parameters_Zuerich
else 
    Parameters_Group
end

%% Plane XY 

q_xy=[phi_z; theta_z];
q_xy_dot=[phi_z_dot;  theta_z_dot];

x_xy=[phi_z, theta_z, phi_z_dot, theta_z_dot];
u_xy=T_z;
 
T_f=-((r_K*r_W*Theta_Axy*sin(alpha)*T_z)/(r_W^2*Theta_Axy+r_K^2*Theta_Wxy*sin(alpha)^2));

f_xy = [q_xy_dot;
        -(((r_W^2*Theta_Axy+r_K^2*Theta_Wxy*sin(alpha)^2)*T_f+r_K*r_W*Theta_Axy*sin(alpha)*T_z)/(r_W^2*Theta_Axy*Theta_K + r_K^2*(Theta_Axy + Theta_K)*Theta_Wxy*sin(alpha)^2));
        -((r_K*sin(alpha)*(r_K*Theta_Wxy*sin(alpha)*T_f +r_W*Theta_K*T_z))/(r_W^2*Theta_Axy*Theta_K+r_K^2*(Theta_Axy+Theta_K)*Theta_Wxy*sin(alpha)^2))];


%% Plane XY - Linearisierung
A_xy_temp=jacobian(f_xy,x_xy);
B_xy_temp=jacobian(f_xy,u_xy);

A_xy=double(subs(A_xy_temp,[x_xy u_xy],[0 0 0 0 0]));
B_xy=double(subs(B_xy_temp,[x_xy u_xy],[0 0 0 0 0]));


C_xy=eye(4);
D_xy=[0;0;0;0];


%Eigenwerte der Systemmatrix berechnen
[lambda_xy]=eig(A_xy);

%Gewichtungsmatrizen für LQR-Regler festlegen

Q_xy = [20 0 0 0; 
        0 100 0 0; 
        0 0 10 0; 
        0 0 0 50];
 R_xy = 200;
 
 
 [K_xy, S_xy, lamda_xy_closed] = lqr(A_xy, B_xy, Q_xy, R_xy);
