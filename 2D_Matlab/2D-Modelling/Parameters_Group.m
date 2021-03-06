%Winkel
alpha = pi/4; 
beta = 0; 


%Allgemeine Konstanten
g=9.81;                 %Erdbeschleunigung
i=353.5;                %Getriebeübersetzung


%Massen
m_s = 0.326;            %Masse Ball --> gewogen
m_w = 0.134*3;          %Masse virtuelles Rad --> Gewicht von 3 Räder + 3 Motoren
m_b = 1.603;             %Masse Gesamtkörper (plus virtuelles Rad)

%Radien und Längen
r_s = 0.08;             %Radius Ball
r_w = 0.03;             %Radius Omni_Wheel --> entspricht auch virtuellem Rad
r_b = 0.0703;           %Radius Körper
l = 0.236;              %Distanz von Schwerpunkt Körper zu Ballmittelpunkt
h = 0.366;              %Distanz von höchsten Punkt Körper zu Ballmittelpunkt

%Trägheitsmomente (berechnet nach ETHZ)
%I_b_x = 0.19862;        %Masseträgheitsmoment Körper (inkl. virtuellem Rad) (yz)-->Formel ETHZ
%I_b_y = 0.19862;        %Masseträgheitsmoment Körper (inkl. virtuellem Rad) (xz)-->Formel ETHZ
%I_b_xy = 0.00396;        %Massenträgheitsmoment Körper (inkl. virtuellem Rad) (xy) --> Formel ETHZ

%Trägheitsmomente (abgelesen SolidEdge)
I_b_x = 0.100;          %Masseträgheitsmoment Körpoer (inkl.virtuellem Rad) (yz)--> SolidEdge
I_b_y = 0.099;          %Masseträgheitsmoment Körpoer (inkl.virtuellem Rad) (xz)--> SolidEdge
I_b_xy = 0.007;          %Masseträgheitsmoment Körpoer (inkl.virtuellem Rad) (xy)--> SolidEdge 
 

%I_w = 0.00236;         %Masseträgheitsmoment virtuelles Rad(yz/xz) --> Formel ETHZ
I_w = 3.579e-3;         %Masseträgheitsmoment virtuelles Rad(yz/xz) --> I_M = 3.8e-8
I_w_xy = 0.0143;        %Masseträgheitsmoment virtuelles Rad(xz) --> Formel ETHZ
I_s = 0.0013909;           %Masseträgheitsmoment Ball --> Formel ETHZ





