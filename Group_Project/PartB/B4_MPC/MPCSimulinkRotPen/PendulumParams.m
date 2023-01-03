%the state space system for the pendulum model

Mp = 0.024;
Lr = 0.085;
Lp = 0.129;
Jr = 5.72E-5;
Dr = 0;
Jp = 3.33E-5;
Dp = 0;
g = 9.81;
Rm = 8.4;
km = 0.042;
kt = 0.042;

Jt = Jr*Jp + Mp*(Lp/2)^2*Jr + Jp*Mp*Lr^2;

A_132 = Mp^2*(Lp/2)^2*Lr*g/Jt;
A_133 = -Dr*(Jp+Mp*(Lp/2)^2)/Jt;
A_134 = -Mp*(Lp/2)*Lr*Dp/Jt;
A_142 = Mp*g*(Lp/2)*(Jr+Mp*Lr^2)/Jt;
A_143 = -Mp*(Lp/2)*Lr*Dr/Jt;
A_144 = -Dp*(Jr+Mp*Lr^2)/Jt;

B_12 = (Jp+Mp*(Lp/2)^2)/Jt;
B_22 =  Mp*(Lp/2)*Lr/Jt;

A = [0 0 1 0;...
     0 0 0 1;...
     0 A_132 A_133 A_134;...
     0 A_142 A_143 A_144];

B = [0; 0; B_12; B_22];

C = [1 0 0 0];

D = [0];

%Actuator Dynamics
B = kt * B/Rm;

A(3,3) = A(3,3)-kt*kt/Rm*B(3);
A(4,3) = A(4,3)-kt*kt/Rm*B(4);

sys1 = ss(A,B,C,D);

G = [zeros(1,1), zeros(1,1), zeros(1,1), zeros(1,1), zeros(1,1);...
     C*B, zeros(1,1), zeros(1,1), zeros(1,1), zeros(1,1);...
     C*A*B, C*B, zeros(1,1), zeros(1,1), zeros(1,1);...
     C*A^2*B, C*A*B, C*B, zeros(1,1), zeros(1,1);...
     C*A^3*B, C*A^2*B, C*A*B, C*B, zeros(1,1)]

H = [C; C*A; C*A^2; C*A^3; C*A^4];

F = [eye(5); -eye(5)];
b = ones([10,1]);

R = eye(5);
Q = eye(5);

S = R+G'*Q*G;
T = (-1 + H*ones([4,1]))'*Q*G;

U_t0 = quadprog(S,T,F,b);
U = [1 0 0 0 0]*U_t0;

