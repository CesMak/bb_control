
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

%Steuer- und Beobachtbarkeit überprüfen
M_S_xy = ctrb(A_xy,B_xy);
M_B_xy = obsv(A_xy,C_xy);

rank_S_xy = rank(M_S_xy);
rank_B_xy = rank(M_B_xy);


 %% Auslegen P-Regler für theta_z
% syms k
k_s=vpa(f_xy(end,1),4); 
k_s=double(subs(k_s,T_z,1));
T_n=0.1;
T_r = 1; 
 %T_n2 = 1/12;
 
%  
G_reg = tf([T_r 1],[T_n 1]);
G_s = tf([k_s],[1 0 0]);
%  
F_o = G_reg*G_s;
%  
F_w = F_o/(1+F_o);
%  
rlocus(F_o);
%  
%  %--> Gain 3.08e08
k_reg = 2.98;
 

 
 
 
 
 