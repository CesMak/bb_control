
%% Plane YZ- Initialisierung der Massenträgheitsmatrix, Gravitations- und Coriolismatrix

%Substitutionen
m_tot=m_s+m_b+m_w;
r_tot=r_s+r_w;
gamma=l*m_b+(r_s+r_w)*m_w;

%Matrizen
M_y=[I_s+m_tot*(r_s)^2+(r_s/r_w)^2*I_w, -((r_s/r_w)^2)*I_w+gamma*r_s*cos(theta_y);
     -((r_s/r_w)^2)*I_w+gamma*r_s*cos(theta_y), I_b+(r_s/r_w)^2*I_w+r_tot^2*m_w];
 
C_y=[-r_s*gamma*sin(theta_y)*(theta_y_dot)^2;
     0];
    
G_y=[0;
     -g*sin(theta_y)*gamma];
  
f_NP1_xz=[(r_s/r_w)*T_y;
       -(1+(r_s/r_w))*T_y];
    
 f_NP2_xz=[0; T_y]; 
 
 f_NP_xz=f_NP1_xz+f_NP2_xz;
 

%% Plane YZ - Nichtlineares System
q_xz=[phi_y; theta_y];
q_xz_dot=[phi_y_dot;  theta_y_dot];

x_xz=[phi_y; theta_y; phi_y_dot; theta_y_dot];
u_xz=T_y;

f_xz = [q_xz_dot; 
     M_y\(f_NP_xz-(C_y+G_y))]; 
 
%% Plane YZ - Linearisierung
A_xz_temp=jacobian(f_xz,x_xz);
B_xz_temp=jacobian(f_xz,u_xz);

A_xz=double(subs(A_xz_temp,[x_xz; u_xz],[0; 0; 0; 0; 0]));
B_xz=double(subs(B_xz_temp,[x_xz; u_xz],[0; 0; 0; 0; 0]));

C_xz = eye(4);
D_xz = [0;0;0;0];


%Eigenwerte der Systemmatrix berechnen
[lambda_xz]=eig(A_xz);

%Steuer- und Beobachtbarkeit überprüfen
M_S_xz = ctrb(A_xz,B_xz);
M_B_xz = obsv(A_xz,C_xz);

rank_S_xz = rank(M_S_xz);
rank_B_xz = rank(M_B_xz);

%Gewichtungsmatrizen für LQR-Regler festlegen

Q_xz = [20 0 0 0; 
        0 100 0 0; 
        0 0 10 0; 
        0 0 0 50];
 R_xz = 200;

 
 [K_xz, S_xz, lamda_xz_closed] = lqr(A_xz, B_xz, Q_xz, R_xz);
 