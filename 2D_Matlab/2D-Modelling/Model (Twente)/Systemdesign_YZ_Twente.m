%% Plane YZ- Initialisierung der Massenträgheitsmatrix, Gravitations- und Coriolismatrix

%Substitutionen
m_tot=m_s+m_w+m_b;
r_tot=r_s+r_w;
gamma=l*m_b+(r_s+r_w)*m_w;

%Matrizen
M_x=[m_tot*r_s^2+I_s+r_s^2/r_w^2*I_w, -(r_s^2/r_w^2)*I_w+gamma*r_s*cos(theta_x);
     -(r_s^2/r_w^2)*I_w+gamma*r_s*cos(theta_x), r_tot^2*m_w+r_s^2/r_w^2*I_w+I_b];
 
C_x=[-r_s*gamma*sin(theta_x)*(theta_x_dot)^2;
     0];
    
G_x=[0;
     -g*sin(theta_x)*gamma];
  
f_ext = [(r_s/r_w)*T_x;
          -(r_s/r_w)*T_x];
      

%% Plane YZ - Nichtlineares System
q_yz=[phi_x; theta_x];
q_yz_dot=[phi_x_dot;  theta_x_dot];

x_yz=[q_yz; q_yz_dot];
u_yz=T_x;

f_yz = [q_yz_dot; 
     M_x\(f_ext-C_x-G_x)]; 
 
%% Plane YZ - Linearisierung
A_yz_temp=jacobian(f_yz,x_yz);
B_yz_temp=jacobian(f_yz,u_yz);


A_yz=double(subs(A_yz_temp,[phi_x theta_x phi_x_dot theta_x_dot T_x],[0 0 0 0 0]));
B_yz=double(subs(B_yz_temp,[phi_x theta_x phi_x_dot theta_x_dot T_x],[0 0 0 0 0]));

C_yz = eye(4);
D_yz = [0;0;0;0];

%% Steuer-und Beobachtbarkeit
%Eigenwerte der Systemmatrix berechnen
[lambda_yz]=eig(A_yz);

%Steuer- und Beobachtbarkeit überprüfen
M_S_yz = ctrb(A_yz,B_yz);
M_B_yz = obsv(A_yz,C_yz);

rank_S_yz = rank(M_S_yz);
rank_B_yz = rank(M_B_yz);

%Gewichtungsmatrizen für LQR-Regler festlegen

Q_yz = [20 0 0 0; 
        0 100 0 0; 
        0 0 10 0; 
        0 0 0 50];
 R_yz = 200;
 
 [K_yz, S_yz, lamda_yz_closed] = lqr(A_yz, B_yz, Q_yz, R_yz);