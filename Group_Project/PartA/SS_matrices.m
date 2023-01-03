m = 0.03;
M = 0.6;
W = 0.14;
D = 0.04;
R = 0.04;
Jw = m*R^2/2;
n = 1;
Jm = 10^(-5);
H = 0.144;
L = H/2;
Jpsi = M*L^2/3;
Jphi = M*(W^2 + D^2)/12;
g = 9.81;
fw = 0;

Kt = 0.317;
Kb = 0.468;
Rm = 6.69;
fm = 0.0022;
beta = n*Kt*Kb/Rm + fm;
alpha = n*Kt/Rm;

E = [((2*m + M)*R^2 + 2*Jw + 2*n^2*Jm)  (M*L*R - 2*n^2*Jm) ;  (M*L*R - 2*n^2*Jm)   (M*L^2 + Jpsi + 2*n^2*Jm)];
I = m*W^2/2 + Jphi + W^2*(Jw + n^2*Jm)/(2*R^2);
J = W^2*(beta + fw)/(2*R^2);
K = W*alpha/(2*R);

A132 = - g*M*L*E(1,2)/det(E); 
A142 = g*M*L*E(1,1)/det(E); 
A133 = -2*[(beta + fw)*E(2,2) + beta*E(1,2)]/det(E);
A143 = 2*[(beta + fw)*E(1,2) + beta*E(1,1)]/det(E);
A134 = 2*beta*[E(2,2) + E(1,2)]/det(E);
A144 = -2*beta*[E(1,1) + E(1,2)]/det(E);
B13 = alpha*[E(2,2) + E(1,2)]/det(E);
B14 = -alpha*[E(1,1) + E(1,2)]/det(E);

%state space system
A1 = [0 0 1 0 ; 0 0 0 1 ; 0 A132 A133 A134 ; 0 A142 A143 A144];
B1 = [0 0 ; 0 0 ; B13 B13 ; B14 B14];
A2 = [0 1 ; 0 -J/I];
B2 = [0 0 ; -K/I K/I];
C1 = [1 0 0 0 ; 0 1 0 0];
C2 = [1 0];
D1 = [0 0 ; 0 0];
D2 = [0 0];

sys1 = ss(A1,B1,C1,D1);

sys2 = ss(A2,B2,C2,D2);


%MPC horizon 5
G1 = [zeros(2,2), zeros(2,2), zeros(2,2), zeros(2,2), zeros(2,2);...
     C1*B1, zeros(2,2), zeros(2,2), zeros(2,2), zeros(2,2);...
     C1*A1*B1, C1*B1, zeros(2,2), zeros(2,2), zeros(2,2);...
     C1*A1^2*B1, C1*A1*B1, C1*B1, zeros(2,2), zeros(2,2);...
     C1*A1^3*B1, C1*A1^2*B1, C1*A1*B1, C1*B1 zeros(2,2)];

H1 = [C1;C1*A1;C1*A1^2;C1*A1^3;C1*A1^4];

save('SS_matrices')