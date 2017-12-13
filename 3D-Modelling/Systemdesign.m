%
%
%
%


%% Static parameters
%
%   contains all kinds of static parameters needed for dynamic calculus
%
%-------------------------------------------------------------------------

%Radius of the ball
r_K;

%Radius of the omniwheel
r_W;

%Radius of the Body A
r_A;

%Distance between the center of the ball and the center of gravity of the
%body
l;

%Mass of the body and omniwheels
m_AW;

%Mass of the ball
m_B;

%Inertia of the body and the omniwheels referenced to frame A
A_Theta_AW = [1 0 0 ; 0 1 0 ; 0 0 1];

%Inertia of the ball reference to frame I
I_Theta_B = [1 0 0 ; 0 1 0 ; 0 0 1];

%Inertia of an omniwheel and motor about the motor axis
Theta_W

%Angles that describe the motor position referenced to frame A
alpha_W = pi/4;
beta_W1 = 0;
beta_W2 = 2/3*pi;
beta_W3 = 4/3*pi;

%Rotational speed vector of the ball referenced to frame L
L_Omega_B = [phi_x_dot ; phi_y_dot; 0];

%Rotational speed of the omniwheels about the motor axis referenced
%to frame A
A_omega_W1 = psi_1;
A_omega_W2 = psi_2;
A_omega_W3 = psi_3;

%Gravity vector
G = [0 ; 0 ; -g];


%% Minimal coordinates
%
%   
%
%-------------------------------------------------------------------------

q = [theta_x ; theta_y ; theta_z ; phi_x ; phi_y];


%% Rotation matrices (I - L - A)
%
%   
%
%-------------------------------------------------------------------------

%Rotation around the x-axis
R_x = [1 0 0 ;
       0 cos(theta_x) -sin(theta_x) ;
       0 sin(theta_x) cos(theta_x)];
   
%Rotation around the y-axis
R_y = [cos(theta_y) 0 sin(theta_y) ;
       0 1 0 ;
       -sin(theta_y) 0 cos(theta_y)];
   
%Rotation around the z-axis
R_z = [cos(theta_z) -sin(theta_z) 0 ;
       sin(theta_z) cos(theta_z) 0 ;
       0 0 1];
   
%Transformation from I to L and vice versa
R_IL = R_z;
R_LI = inv(R_z);

%Transformation from I to A and vice versa
R_IA = R_IL*R_y*R_x;
R_AI = inv(R_IA);

%% Rotational dynamics
%
%   
%
%-------------------------------------------------------------------------
%Rotation of the ball referenced to I
I_Omega_K = R_IL*L_Omega_K;

%Jacobian Matrix
J = [1 0 -sin(theta_y) ;
     0 cos(theta_x) sin(theta_x)*cos(theta_y) ; 
     0 -sin(theta_x) cos(theta_x)*cos(theta_y)];
 
%Derivative of the Tait-Bryan angles
Theta_dot = [theta_x_dot ; theta_y_dot ; theta_z_dot];

%Rotation of the body referenced to frame A
A_Omega_A = J*Theta_dot;


%% Binding equations



