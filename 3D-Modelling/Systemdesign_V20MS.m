%
%
%
%
clc;
clear all;

%% Static parameters
%
%   contains all kinds of static parameters needed for dynamic calculus
%
%-------------------------------------------------------------------------

%Radius of the ball
r_B = 0.07;

%Radius of the omniwheel
r_W = 0.03;

%Radius of the Body A
r_A = 0.00703;

%Distance between the center of the ball and the center of gravity of the
%body
l = 0.24045;

%Mass of the body
m_A = 1.646;

%Mass of one omniwheel including motor
m_W = 0.118;

%Mass of the body and omniwheels
m_AW = 1.764;

%Mass of the ball
m_B = 0.4;

%Inertia of the body and the omniwheels referenced to frame A (SolidEdge)
A_Theta_AW = [0.020, 0, 0 ; 0, 0.005, -0.001 ; 0, 0, 0.019];

%Inertia of the ball reference to frame I (Solid Edge)
I_Theta_B = [0.00195, 0, 0 ; 0, 0.00195 0 ; 0, 0, 0.00195];

%Inertia of an omniwheel and motor about the motor axis (Calculation and Assumption)
%
%Mit Florian abklären ob richtig
%
Theta_W = 0.16656;

%Angles that describe the motor position referenced to frame A
alpha_W = pi/4;
beta_W1 = 0;
beta_W2 = 2/3*pi;
beta_W3 = 4/3*pi;

%Rotational speed vector of the ball referenced to frame L
syms phi_x phi_y phi_x_dot phi_y_dot;
L_Omega_B = [phi_x_dot ; phi_y_dot; 0];

%Rotational speed of the omniwheels about the motor axis referenced
%to frame A
syms psi_1_dot psi_2_dot psi_3_dot;
A_omega_W1 = psi_1_dot;
A_omega_W2 = psi_2_dot;
A_omega_W3 = psi_3_dot;

%Gravity vector referenced to frame I
I_G = [0 ; 0 ; -9.81];


%% Minimal coordinates
%
%   
%
%-------------------------------------------------------------------------
syms theta_x theta_y theta_z phi_x phi_y;
q = [theta_x ; theta_y ; theta_z ; phi_x ; phi_y];


%% Rotation matrices (I - L - A)
%
%   
%
%-------------------------------------------------------------------------

%Rotation around the x-axis
R_x = [1, 0, 0 ;
       0, cos(theta_x), -sin(theta_x) ;
       0, sin(theta_x), cos(theta_x)];
   
%Rotation around the y-axis
R_y = [cos(theta_y), 0, sin(theta_y) ;
       0, 1, 0 ;
       -sin(theta_y), 0, cos(theta_y)];
   
%Rotation around the z-axis
R_z = [cos(theta_z), -sin(theta_z), 0 ;
       sin(theta_z), cos(theta_z), 0 ;
       0, 0, 1];
   
%Transformation from I to L and vice versa
R_IL = R_z;
R_LI = inv(R_z);

%Transformation from I to A and vice versa
R_IA = R_z*R_y*R_x;
R_AI = inv(R_IA);

%Transformation form L to A and vice versa
R_LA = R_y*R_x;
R_AL = inv(R_LA);

%% Rotational dynamics
%
%   
%
%-------------------------------------------------------------------------
%Rotation of the ball referenced to I
I_Omega_B = R_LI*L_Omega_B;

%Jacobian Matrix
J = [1, 0, -sin(theta_y) ;
     0, cos(theta_x), sin(theta_x)*cos(theta_y) ; 
     0, -sin(theta_x), cos(theta_x)*cos(theta_y)];
 
%Derivative of the Tait-Bryan angles
syms theta_x_dot theta_y_dot theta_z_dot;
Theta_dot = [theta_x_dot ; theta_y_dot ; theta_z_dot];

%Rotation of the body referenced to frame A
A_Omega_A = simplify(J*Theta_dot);


%% Omniwheel equations needed for binding equations
%
%   
%
%-------------------------------------------------------------------------

%Rotational vector from the intersection of the motor asis M to the center point of
%each wheel 
A_R_MW1 = [cos(beta_W1)*sin(alpha_W) ; sin(alpha_W)*sin(beta_W1) ; -cos(alpha_W)];
A_R_MW2 = [cos(beta_W2)*sin(alpha_W) ; sin(alpha_W)*sin(beta_W2) ; -cos(alpha_W)];
A_R_MW3 = [cos(beta_W3)*sin(alpha_W) ; sin(alpha_W)*sin(beta_W3) ; -cos(alpha_W)];

%Translational vector from the center of the ball to each contactpoint
%with each omniwheel referenced to frame A
A_T_PK1 = [r_B*sin(alpha_W)*cos(beta_W1) ; r_B*sin(alpha_W)*sin(beta_W1) ; r_B*cos(alpha_W)];
A_T_PK2 = [r_B*sin(alpha_W)*cos(beta_W2) ; r_B*sin(alpha_W)*sin(beta_W2) ; r_B*cos(alpha_W)];
A_T_PK3 = [r_B*sin(alpha_W)*cos(beta_W3) ; r_B*sin(alpha_W)*sin(beta_W3) ; r_B*cos(alpha_W)];

%Translational vector from the center of the ball P to the center of gravity
%G of the body referenced to frame A
A_T_PG = [0 ; 0 ; 1];


%Absolute rotation speed of each omniwheel about the motor axis referenced
%to frame A
A_Omega_W1 = simplify(A_omega_W1 + dot(A_R_MW1,A_Omega_A));
A_Omega_W2 = simplify(A_omega_W2 + dot(A_R_MW2,A_Omega_A));
A_Omega_W3 = simplify(A_omega_W3 + dot(A_R_MW3,A_Omega_A));


%Direction of the tangential speed of the rotation of each wheel referenced
%to frame A
A_vt_W1 = [-sin(beta_W1) ; cos(beta_W1) ; 0]
A_vt_W2 = [-sin(beta_W2) ; cos(beta_W2) ; 0]
A_vt_W3 = [-sin(beta_W3) ; cos(beta_W3) ; 0]


%% Ball equations needed for binding equations
%
%   
%
%-------------------------------------------------------------------------

%Rotaional speed of the ball relative to the body A referenced to frame A
%
%Simplification needed
%
A_Omega_B = R_LA*L_Omega_B - A_Omega_A
%A_Omega_B = R_AI*L_Omega_B - A_Omega_A;



%% Binding equations regarding the ball and the omniwheels
%
%   
%
%-------------------------------------------------------------------------

%The speed of the surface of the ball has to match each tangetial omniwheel
%speed, therefore three equations evolve

%E_1 = dot(cross(A_Omega_B , A_T_PK1),A_vt_W1) == A_omega_W1*r_W;
E_1_precalc = cross(A_Omega_B , A_T_PK1);
E_1 = E_1_precalc(2) == A_omega_W1*r_W;
E_2_precalc = cross(A_Omega_B , A_T_PK2);
%E_2 = -E_2_precalc(1)*0.866 - 0.5*E_2_precalc(2) == A_omega_W2*r_W
E_2_new = simplify(-E_2_precalc(1)*0.866 - 0.5*E_2_precalc(2))
E_3_precalc = cross(A_Omega_B , A_T_PK3);
E_3 = 0.866*E_3_precalc(1) - 0.5*E_3_precalc(2) == A_omega_W3*r_W;

%Solving upper equations for psi_1_dot, psi_2_dot, psi_3_dot
bindingSol = solve([E_1 , E_2_new , E_3] , [psi_1_dot , psi_2_dot , psi_3_dot])
%bindingSol = solve([E_1, E_2, E_3], psi_1_dot);


%% Ball equations
%
%   
%
%-------------------------------------------------------------------------

%Vector from the ground to the center of the ball P referenced to frame I
T_BP = [0 ; 0 ; r_B];

%Speed vector of the Ball in point P referenced to frame I
v_B = simplify(cross(I_Omega_B, T_BP));


%% Energies
%
%   
%
%-------------------------------------------------------------------------

%
%
%Ball 
%
%kinetic energy
L_Omega_B
Kin_B = 1/2 * m_B * dot(v_B,v_B) + 1/2 * L_Omega_B * I_Theta_B * L_Omega_B;
%potential energy
Pot_B = 0;


%
%
%Body A
%
%kinetic energy
Kin_A = simplify(1/2 * m_AW * dot(v_B,v_B) + m_AW * dot(R_IA*v_B,cross(A_Omega_A, A_T_PG)) + 1/2 * A_Omega_A * A_Theta_AW * A_Omega_A);
%potential energy
Pot_A = -m_AW * R_IA * I_G * A_T_PG;


%
%
%Wheels
%
%kin energy of each wheel
Kin_W1 = simplify(1/2 * Theta_W * A_Omega_W1 * A_Omega_W1);
Kin_W2 = simplify(1/2 * Theta_W * A_Omega_W1 * A_Omega_W2);
Kin_W3 = simplify(1/2 * Theta_W * A_Omega_W1 * A_Omega_W3);








