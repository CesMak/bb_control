%Winkel
alpha = pi/4; 
beta = (2*pi)*2/3; 


%Allgemeine Konstanten
g=9.81; 
i=353.5;


%Massen
m_s = 0.325;
m_w = 0.134; 
m_b = 1.603;

%Radien und Längen
r_s = 0.08; 
r_w = 0.03;
r_b = 0.0703; 
l = 0.220;
h = 0.353; 

%Trägheitsmomente
I_b= 0.088;
I_b_y = 0.087;
I_b_z = 0.007; 
I_w = 2/3*cos(alpha)^2*(28.35e-6+i^2*3.33e-6);  
I_s = 0.001287;  



