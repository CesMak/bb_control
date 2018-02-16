%Winkel
alpha = pi/4; 
beta = 0; 


%Allgemeine Konstanten
g=9.81;                 %Erdbeschleunigung
i=353.5;                %Getriebe�bersetzung


%Massen
m_s = 0.326;            %Masse Ball --> gewogen
m_w = 0.134*3;          %Masse virtuelles Rad --> Gewicht von 3 R�der + 3 Motoren
m_b = 1.603;             %Masse Gesamtk�rper (plus virtuelles Rad)

%Radien und L�ngen
r_s = 0.08;             %Radius Ball
r_w = 0.03;             %Radius Omni_Wheel --> entspricht auch virtuellem Rad
r_b = 0.0703;           %Radius K�rper
l = 0.236;              %Distanz von Schwerpunkt K�rper zu Ballmittelpunkt
h = 0.366;              %Distanz von h�chsten Punkt K�rper zu Ballmittelpunkt

%Tr�gheitsmomente (berechnet nach ETHZ)
%I_b_x = 0.19862;        %Massetr�gheitsmoment K�rper (inkl. virtuellem Rad) (yz)-->Formel ETHZ
%I_b_y = 0.19862;        %Massetr�gheitsmoment K�rper (inkl. virtuellem Rad) (xz)-->Formel ETHZ
%I_b_xy = 0.00396;        %Massentr�gheitsmoment K�rper (inkl. virtuellem Rad) (xy) --> Formel ETHZ

%Tr�gheitsmomente (abgelesen SolidEdge)
I_b_x = 0.100;          %Massetr�gheitsmoment K�rpoer (inkl.virtuellem Rad) (yz)--> SolidEdge
I_b_y = 0.099;          %Massetr�gheitsmoment K�rpoer (inkl.virtuellem Rad) (xz)--> SolidEdge
I_b_xy = 0.007;          %Massetr�gheitsmoment K�rpoer (inkl.virtuellem Rad) (xy)--> SolidEdge 
 

%I_w = 0.00236;         %Massetr�gheitsmoment virtuelles Rad(yz/xz) --> Formel ETHZ
I_w = 3.579e-3;         %Massetr�gheitsmoment virtuelles Rad(yz/xz) --> I_M = 3.8e-8
I_w_xy = 0.0143;        %Massetr�gheitsmoment virtuelles Rad(xz) --> Formel ETHZ
I_s = 0.0013909;           %Massetr�gheitsmoment Ball --> Formel ETHZ





