%Winkel
alpha = pi/4; 
beta = (2*pi)*2/3; 


%Allgemeine Konstanten
g=9.81; 
i=353.5;


%Massen
m_s = 0.325;
m_w = 0.435; 
m_b = 1.273;

%Radien und Längen
r_s = 0.08; 
r_w = 0.03;
r_b = 0.0703; 
l = 0.220;
h = 0.353; 

%Trägheitsmomente
I_b= 0.088;
Theta_A_yy = 0.087;
Theta_A_zz = 0.007; 
I_w = 2/3*cos(alpha)^2*(28.35e-6+i^2*5e-6); 
%Theta_Wxy = Theta_W * 3/(2*cos(alpha)^2); 
I_s = 0.001287;  
%Theta_Axy = 0.092; 

%Theta_A_xx = 0.25*(m_A+m_W)*r_K^2+0.5*(m_A+m_W)*l^2+(m_A+m_W)*h^2;
%Theta_A_yy = Theta_A_xx; 
%Theta_A_zz = 0.5*(m_A+m_W)*r_A^2; 




