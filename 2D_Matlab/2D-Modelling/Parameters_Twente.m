
%Massen
m_s = 3.2;      %Masse Ball
m_ow = 0.555;   %OmniWheel-including clamping bush and flange
m_w = 0.995;    %Masse virtuelles Rad 
m_b = 7.135;    %Masse Körper

%Radien und Längen
r_s = 0.115; %Radius Ball 
r_w = 0.050; %Radius virtuelles Rad
r_b = 0.1;   %Radius Körper
l = 0.405;   %Länge Ballmittelpunkt zu Schwerpunkt Körper

%Trägheitsmomente
I_s = 0.0265;       %Trägheitsmoment Ball
I_w = 0.00190;      %Massenträgheitsmoment Rad um xx und yy
I_w_xy = 0.00381;   %Massenträgheitsmoment Rad um zz
I_b = 2.40;         %Massenträgheitsmoment Körper um xx und yy 
I_b_xy = 0.0476;    %Massenträgheitsmoment Körper um zz 

%Allgemeine Konstanten
g=9.81; 
%i=26; 

%Winkel
alpha = pi/4;   %Angle the omni-wheels make with the top of the ball
beta = 0;       %Angle that determines the horizontal position of the omni_wheels





