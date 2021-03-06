%% Energies
%Energie Ball
T_s = 0.5*m_s*r_s^2*phi_dot^2+0.5*I_s*phi_dot^2;
V_s = 0; 

%Energie Body
T_b = 0.5*m_b*((r_s*phi_dot)^2+2*r_s*l*phi_dot*theta_dot*cos(theta)+l^2*theta_dot^2)+0.5*(I_b)*theta_dot^2;
V_b = m_b*g*l*cos(theta);

% Energie Wheel
T_w = 0.5*m_w*((r_s*phi_dot)^2+2*r_s*(r_s+r_w)*phi_dot*theta_dot*cos(theta)+(r_s+r_w)^2*theta_dot^2)+0.5*I_w*((r_s/r_w)*(phi_dot-theta_dot)-theta_dot)^2;
V_w = m_w*g*(r_s+r_w)*cos(theta);

% Non-Potential Forces
f_phi = (r_s/r_w)*F;
f_theta= -(r_s/r_w)*F;

% Total Energies
T = T_s+T_b+T_w;
V = V_s+V_b+V_w;

%% Langrange
L = T-V;

%dL/dphi
L_phi = jacobian(L,phi);

%dL/ddphi
L_phi_dot = jacobian(L,phi_dot);

%dL/dtheta
L_theta = jacobian(L,theta);

%dL/ddtheta
L_theta_dot = jacobian(L,theta_dot);


%Variablen ohne t mit t ersetzen

L_phi_dot_t = subs(L_phi_dot,{phi, phi_dot, theta, theta_dot},{'phi(t)','phi_dot(t)','theta(t)','theta_dot(t)'});

L_theta_dot_t = subs(L_theta_dot,{phi, phi_dot, theta, theta_dot},{'phi(t)','phi_dot(t)','theta(t)','theta_dot(t)'});

%Berechung Zeitableitungen
dL_phi_dot_t = diff(L_phi_dot_t,t);
dL_theta_dot_t = diff(L_theta_dot_t,t);

%% Substitution

%Variablen mit t zu ohne t
Var_t = {'phi(t)','phi_dot(t)','diff(phi(t),t)','diff(phi_dot(t),t)','theta(t)','theta_dot(t)','diff(theta(t),t)','diff(theta_dot(t),t)'};
Var_ot = [phi,phi_dot,phi_dot,phi_dotdot,theta,theta_dot,theta_dot,theta_dotdot];

dL_phi_dot_t = subs(dL_phi_dot_t,Var_t,Var_ot);
dL_theta_dot_t = subs(dL_theta_dot_t,Var_t,Var_ot);

%Berechung
Sol = solve([dL_phi_dot_t-L_phi==f_phi, dL_theta_dot_t-L_theta==f_theta],[phi_dotdot, theta_dotdot]);