
%Massen
m_s = 3.2; %Ball
%m_ow = 0.555; %OmniWheel-including clamping bush and flange
m_w = 0.995; %mass of a virtual actuating wheel 
m_b = 7.135; %Body

%Radien und Längen
r_s = 0.115; %Radius Ball 
r_w = 0.050; %Radius virtuel wheel
r_b = 0.1; 
l = 0.405; %Distance COM Ball to COM Body 

%Trägheitsmomente
I_s = 0.0265; %2.65e-2; % Moment of Inertia of the Ball
I_w = 0.00190; %Moment of Inertia of the virtual actuatin wheel in yz/xz 
I_w_xy = 0.00381; %Moment of Inertia of the virtual actuating wheel in xy-plane 
I_b = 2.40; %Moment of Inertia around yz/xz 
I_b_xy = 0.0476; %Moment of Inertia of the body in xy plane 

%Allgemeine Konstanten
g=9.81; 
i=26; 

%Winkel
alpha = pi/4; %Angle the omni-wheels make with the top of the ball
beta = 0; %Angle that determines the horizontal position of the omni_wheels





