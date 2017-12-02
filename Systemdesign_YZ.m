clc;
clear all;

Parameter_flag = 1; 
%syms m_K m_W m_A r_K r_W r_A l Theta_K Theta_W Theta_Wxy Theta_A Theta_Axy phi_x phi_x_dot theta_x theta_x_dot T_x g i 
syms phi_z phi_z_dot phi_x phi_x_dot theta_z theta_z_dot theta_x theta_x_dot T_x T_y T_z T_f 

if Parameter_flag == 0
    Parameters_Zuerich
else 
    Parameters_Group
end



%% Plane YZ- Initialisierung der Massenträgheitsmatrix, Gravitations- und Coriolismatrix

%Substitutionen
m_tot=m_K+m_A+m_W;
r_tot=r_K+r_W;
gamma=l*m_A+(r_K+r_W)*m_W;

%Matrizen
M_x=[m_tot*(r_K)^2+Theta_K+(r_K/r_W)^2*Theta_W, -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_x);
     -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_x), ((r_tot)^2/(r_W)^2)*Theta_W+Theta_A+m_A*l^2+m_W*(r_tot)^2];
 
C_x=[-r_K*gamma*sin(theta_x)*(theta_x_dot)^2;
     0];
    
G_x=[0;
     -g*sin(theta_x)*gamma];
  
f_NP1=[(r_K/r_W)*T_x;
       -(1+(r_K/r_W))*T_x];
    
 f_NP2=[0; T_x]; 
 
 f_NP=f_NP1+f_NP2;
 

%% Plane YZ - Nichtlineares System
q_yz=[phi_x; theta_x];
q_yz_dot=[phi_x_dot;  theta_x_dot];

x_yz=[phi_x, theta_x, phi_x_dot, theta_x_dot];
u_yz=T_x;

f_yz = [q_yz_dot; 
     M_x\(f_NP-(C_x+G_x))]; 
 
%% Plane YZ - Linearisierung
A_yz_temp=jacobian(f_yz,x_yz);
B_yz_temp=jacobian(f_yz,u_yz);


A_yz=double(subs(A_yz_temp,[x_yz u_yz],[0 0 0 0 0]));
B_yz=double(subs(B_yz_temp,[x_yz u_yz],[0 0 0 0 0]));

C_yz = eye(4);
D_yz = [0;0;0;0];


%Eigenwerte der Systemmatrix berechnen
[lambda_yz]=eig(A_yz);

%Gewichtungsmatrizen für LQR-Regler festlegen

Q_yz = [20 0 0 0; 
        0 100 0 0; 
        0 0 10 0; 
        0 0 0 50];
 R_yz = 200;
 
 [K_yz, S_yz, lamda_yz_closed] = lqr(A_yz, B_yz, Q_yz, R_yz);
 
 %Berechunung Vorfilter
 A_yz_closed = A_yz - B_yz*K_yz; 
 
 %F_yz = -inv(C_yz*inv(A_yz-B_yz*K_yz)*B_yz); 
 