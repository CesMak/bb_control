%RealTorques Plotten
figure(); 
t=Real_Torques_Full.time; 

%Plot T1
subplot(3,1,1);
plot(t,Real_Torques_Full.signals(1).values);
title('Motor T_{1}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

%Plot T2
subplot(3,1,2); 
plot(t,Real_Torques_Full.signals(2).values);
title('Motor T_{2}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

%Plot T3
subplot(3,1,3); 
plot(t,Real_Torques_Full.signals(3).values);
title('Motor T_{3}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

suptitle('Reale Drehmomente mit Rauschen');

%Virtuelle Drehmomente Plotten
figure(); 
t=Virtual_Torques_FULL.time; 

%Plot T1
subplot(3,1,1);
plot(t,Virtual_Torques_FULL.signals(1).values);
title('T_{x}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

%Plot T2
subplot(3,1,2); 
plot(t,Virtual_Torques_FULL.signals(2).values);
title('T_{y}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

%Plot T3
subplot(3,1,3); 
plot(t,Virtual_Torques_FULL.signals(3).values);
title('T_{z}');
xlabel('t in s');
ylabel('M in Nm');
grid on;

suptitle('Virtuelle Drehmomente mit Rauschen');

%Angles
figure(); 
t=Phi_Struct.time; 

%Plot Theta_x
t=Theta_Struct.time;
subplot(4,1,1); 
plot(t,Theta_Struct.signals(1).values);
title('Winkel $$\theta_{x}$$','Interpreter','latex');
xlabel('t in s');
ylabel('$$\theta_{x}$$ in rad','Interpreter','latex');
grid on;

%Plot Theta_x
subplot(4,1,2); 
plot(t,Theta_Struct.signals(2).values);
title('Winkelgeschwindigkeit $$\dot{\theta_{x}}$$','Interpreter','latex');
xlabel('t in s');
ylabel('$$\dot{\theta_{x}}$$ in rad/s','Interpreter','latex');
grid on;

%Plot Theta_y
t=Theta_Struct.time;
subplot(4,1,3); 
plot(t,Theta_Struct.signals(3).values);
title('Winkel $$\theta_{y}$$','Interpreter','latex');
xlabel('t in s');
ylabel('$$\theta_{y}$$ in rad','Interpreter','latex');
grid on;

%Plot Theta_y
subplot(4,1,4); 
plot(t,Theta_Struct.signals(4).values);
title('Winkelgeschwindigkeit $$\dot{\theta_{y}}$$','Interpreter','latex');
xlabel('t in s');
ylabel('$$\dot{\theta_{y}}$$ in rad/s','Interpreter','latex');
grid on;


suptitle('Winkel und Winkelgeschwindigkeiten mit Rauschen ');
%suptitle(sprintf('Reale Winkel- und Winkelgeschwindigkeiten mit Rauschen;\n $$y_{0} = \frac{\PI}{32}$$;\n K=[-0.0316  -14.4092   -0.2409   -3.4304]'));

