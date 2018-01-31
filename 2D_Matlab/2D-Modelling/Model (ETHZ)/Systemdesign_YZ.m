
%% Plane YZ- Initialisierung der Massenträgheitsmatrix, Gravitations- und Coriolismatrix

%Substitutionen
m_tot=m_s+m_b+m_w;
r_tot=r_s+r_w;
gamma=l*m_b+(r_s+r_w)*m_w;

%Matrizen
M_x=[m_tot*(r_s)^2+I_s+(r_s/r_w)^2*I_w, -(r_s/r_w^2)*r_tot*I_w+gamma*r_s*cos(theta_x);
     -(r_s/r_w^2)*r_tot*I_w+gamma*r_s*cos(theta_x), r_tot^2/r_w^2*I_w+I_b_x+m_b*l^2+m_w*r_tot^2];
 
C_x=[-r_s*gamma*sin(theta_x)*(theta_x_dot)^2;
     0];
    
G_x=[0;
     -g*sin(theta_x)*gamma];
  
f_NP1=[(r_s/r_w)*T_x;
       -(1+(r_s/r_w))*T_x];
    
 f_NP2=[0; T_x]; 
 
 f_NP=f_NP1+f_NP2;
 

%% Plane YZ - Nichtlineares System
q_yz=[phi_x; theta_x];
q_yz_dot=[phi_x_dot;  theta_x_dot];

x_yz=[phi_x; theta_x; phi_x_dot; theta_x_dot];
u_yz=T_x;

f_yz = [q_yz_dot; 
     M_x\(f_NP-(C_x+G_x))]; 
 
%% Plane YZ - Linearisierung
A_yz_temp=jacobian(f_yz,x_yz);
B_yz_temp=jacobian(f_yz,u_yz);


A_yz=double(subs(A_yz_temp,[x_yz; u_yz],[0; 0; 0; 0; 0]));
B_yz=double(subs(B_yz_temp,[x_yz; u_yz],[0; 0; 0; 0; 0]));

C_yz = eye(4);
D_yz = [0;0;0;0];


%Eigenwerte der Systemmatrix berechnen
[lambda_yz]=eig(A_yz);

%Steuer- und Beobachtbarkeit überprüfen
M_S_yz = ctrb(A_yz,B_yz);
M_B_yz = obsv(A_yz,C_yz);

rank_S_yz = rank(M_S_yz);
rank_B_yz = rank(M_B_yz);

%Gewichtungsmatrizen für LQR-Regler festlegen

Q_yz = [20 0 0 0; 
        0 100*q2_lqr 0 0; 
        0 0 10*q3_lqr 0; 
        0 0 0 50*q4_lqr];
 R_yz = 200*r_lqr;
 
 [K_yz, S_yz, lamda_yz_closed] = lqr(A_yz, B_yz, Q_yz, R_yz);
 
 