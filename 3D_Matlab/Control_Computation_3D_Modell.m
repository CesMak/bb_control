%Matrizen
A = [[0             1           0           0           0           0           0           0           0           0], 
     [46.827        0           0           0           0           0           0           0           0           0], 
     [0             0           0           1           0           0           0           0           0           0], 
     [0             0         48.808        0           0           0           0           0           0           0], 
     [0             0           0           0           0           1           0           0           0           0], 
     [0             0        0.00873        0           0           0           0           0           0           0], 
     [0             0           0           0           0           0           0           1           0           0], 
     [57.336        0           0           0           0           0           0           0           0           0], 
     [0             0           0           0           0           0           0           0           0           1], 
     [0             0         59.761        0           0           0           0           0           0           0]];


 B = [[0            0           0],
      [-1.458     0.0452      -1.458], 
      [0            0           0], 
      [-2.632     -1.728      -0.823], 
      [0            0           0], 
      [-0.338     -0.338      -0.338], 
      [0            0           0], 
      [-2.976     2.437       -2.976], 
      [0            0           0], 
      [-5.286     -2.115       1.055]];
  
  C = eye(10,10);
 
 
%Eigenwerte der Systemmatrix berechnen
[lambda]=eig(A)

%Steuer- und Beobachtbarkeit überprüfen
M_S = ctrb(A,B)
M_B = obsv(A,C)

rank_S = rank(M_S)
rank_B = rank(M_B)

%Gewichtungsmatrizen für LQR-Regler festlegen

Q = diag([100, 50, 100, 50, 40, 20, 20, 10, 20, 10]);
 R = diag([100, 100, 100]);
 
 [K, S, lamda_closed] = lqr(A, B, Q, R)
 
 %K = transpose(K)