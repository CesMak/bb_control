%Winkel
alpha = pi/4; 
beta = (-2*pi)/3; 
beta_1 = 0; 
beta_2 = (2*pi)/3;
beta_3 = (4*pi)/3;

%Allgemeine Konstanten
g=9.81; 
i=353.5;


%Massen
m_K = 0.325;
m_W = 0.435; 
m_A = 1.273;

%Radien und Längen
r_K = 0.08; 
r_W = 0.03;
r_A = 0.0703; 
l = 0.220;
h = 0.353; 

%Trägheitsmomente
Theta_A_xx = 0.088;
Theta_A_yy = 0.087;
Theta_A_zz = 0.007; 
Theta_W = 2/3*cos(alpha)^2*(28.35e-6+i^2*0.444e-6); 
Theta_Wxy = Theta_W * 3/(2*cos(alpha)^2); 
Theta_K = 0.001287;  
%Theta_Axy = 0.092; 

%Theta_A_xx = 0.25*(m_A+m_W)*r_K^2+0.5*(m_A+m_W)*l^2+(m_A+m_W)*h^2;
%Theta_A_yy = Theta_A_xx; 
%Theta_A_zz = 0.5*(m_A+m_W)*r_A^2; 




