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
    Parameters_Geschaetzt
end


% Arbeitspunkte
x_AP = [0;0;0;0;0;0];
u_AP =[0;0;0];

% Anfangsbedingungen
x_0 = [pi/32 0 pi/32 0 pi/32 0];


%% theta_x dot dot 
%Substitutionen
m_tot=m_K+m_A+m_W;
r_tot=r_K+r_W;
gamma=l*m_A+(r_K+r_W)*m_W;

%Matrizen
M_x=[m_tot*(r_K)^2+Theta_K+(r_K/r_W)^2*Theta_W, -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_x);
     -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_x), ((r_tot)^2/(r_W)^2)*Theta_W+Theta_A_xx+m_A*l^2+m_W*(r_tot)^2];
 
C_x=[-r_K*gamma*sin(theta_x)*(theta_x_dot)^2;
     0];
    
G_x=[0;
     -g*sin(theta_x)*gamma];
  
f_NP1=[(r_K/r_W)*T_x;
       -(1+(r_K/r_W))*T_x];
    
 f_NP2=[0; T_x]; 
 
 f_NP=f_NP1+f_NP2;
 
 
 f_yz_temp = M_x\(f_NP-(C_x+G_x));
 
 f_yz = f_yz_temp(2,:); 
 
 
 %% theta_y_dotdot 
 
 %Substitutionen
m_tot=m_K+m_A+m_W;
r_tot=r_K+r_W;
gamma=l*m_A+(r_K+r_W)*m_W;

%Matrizen
M_y=[m_tot*(r_K)^2+Theta_K+(r_K/r_W)^2*Theta_W, -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_y);
     -(r_K/(r_W)^2)*r_tot*Theta_W+gamma*r_K*cos(theta_y), ((r_tot)^2/(r_W)^2)*Theta_W+Theta_A_yy+m_A*l^2+m_W*(r_tot)^2];
 
C_y=[-r_K*gamma*sin(theta_y)*(theta_y_dot)^2;
     0];
    
G_y=[0;
     -g*sin(theta_y)*gamma];
  
f_NP1_xz=[(r_K/r_W)*T_y;
       -(1+(r_K/r_W))*T_y];
    
 f_NP2_xz=[0; T_y]; 
 
 f_NP_xz=f_NP1_xz+f_NP2_xz;
 
 f_xz_temp =  M_y\(f_NP_xz-(C_y+G_y)); 
 
 f_xz = f_xz_temp(2,:); 
 
 %% Theta_z_dotdot
 
T_f=-((r_K*r_W*Theta_A_zz*sin(alpha)*T_z)/(r_W^2*Theta_A_zz+r_K^2*Theta_Wxy*sin(alpha)^2)); 
f_xy = -((r_K*sin(alpha)*(r_K*Theta_Wxy*sin(alpha)*T_f +r_W*Theta_K*T_z))/(r_W^2*Theta_A_zz*Theta_K+r_K^2*(Theta_A_zz+Theta_K)*Theta_Wxy*sin(alpha)^2));


%% Zustand

sys_nl = [theta_x_dot;f_yz;theta_y_dot; f_xz; theta_z_dot; f_xy]; 
x = [theta_x, theta_x_dot, theta_y, theta_y_dot, theta_z, theta_z_dot]; 
u = [T_x, T_y, T_z]; 
f=[f_yz;f_xz;f_xy];
A_temp=jacobian(sys_nl,x);
B_temp=jacobian(sys_nl,u);


A=double(subs(A_temp,[x u],[0 0 0 0 0 0 0 0 0]));
B=double(subs(B_temp,[x u],[0 0 0 0 0 0 0 0 0]));
C= [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];
D= [0;0;0];

%Eigenwerte der Systemmatrix berechnen
[lambda]=eig(A);

M_S = ctrb(A,B);

rank_S = rank(M_S);


%% Regler LQR
%Gewichtungsmatrizen für LQR-Regler festlegen

Q = [5 0 0 0 0 0;
        0 10 0 0 0 0; 
        0 0 5 0 0 0;
        0 0 0 10 0 0;
        0 0 0 0 0.1 0;
        0 0 0 0 0 0.1];
        
R=[20 0 0;
    0 20 0;
    0 0 20];
 
[K, S, lamda_closed] = lqr(A, B, Q, R);

%% Regler Polplatzierung

pole=[-1; -0.5; -1; -0.5; -1.; -0.5]; 
K_place = place(A,B,pole);




