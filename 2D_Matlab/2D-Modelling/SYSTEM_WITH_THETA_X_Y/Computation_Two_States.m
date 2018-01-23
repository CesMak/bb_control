%% Clean-up Workspace
clc;
clear all;

%% Load Model Parameter
Parameter_flag = 1; 
%syms m_K m_W m_A r_K r_W r_A l Theta_K Theta_W Theta_Wxy Theta_A Theta_Axy phi_x phi_x_dot theta_x theta_x_dot T_x g i 
syms phi_z phi_z_dot phi_x phi_x_dot phi_y phi_y_dot theta_z theta_z_dot theta_x theta_x_dot theta_y theta_y_dot T_x T_y T_z T_f 

if Parameter_flag == 0
    Parameters_Zuerich
    Theta_A_xx = Theta_A;
    Theta_A_yy = Theta_A;
elseif Parameter_flag == 2
    Parameters_Group
elseif Parameter_flag == 1
    Parameters_Geschaetzt
end

%% Bewegungsgleichung für Theta_x --> Theta_x_dotdot=........

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
 
 
%% Bewegungsgleichung für Theta_y --> Theta_y_dotdot=...... 
 
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
 
%% Zustand

sys_nl = [theta_x_dot; f_yz; theta_y_dot; f_xz]; 
x = [theta_x theta_x_dot theta_y theta_y_dot]; 
u = [T_x T_y T_z]; 

A_temp=jacobian(sys_nl,x);
B_temp=jacobian(sys_nl,u);


%Arbeitspunkt und Anfangspunkte
% Arbeitspunkte
x_AP = [0 0 0 0];
u_AP =[0 0 0];

% Anfangsbedingungen
x_0 = [0 0 0 0];

A=double(subs(A_temp,[x u],[x_AP u_AP]));
B=double(subs(B_temp,[x u],[x_AP u_AP]));
C= [1 0 0 0;
    0 0 1 0 ];
D= [0;0;0];

%Eigenwerte der Systemmatrix berechnen
[lambda]=eig(A);

M_S = ctrb(A,B);

rank_S = rank(M_S);


%% Regler LQR
%Gewichtungsmatrizen für LQR-Regler festlegen

Q = [10 0 0  0;
     0 5000 0 0; 
     0 0 10  0;
     0 0 0 5000];
        
R=[200000 0 0;
    0 200000 0;
    0 0 200000];
 
[K, S, lamda_closed] = lqr(A, B, Q, R);

%% Regler Polplatzierung

pole=[-10; -5; -10; -5]; 
K_place = place(A,B,pole);




