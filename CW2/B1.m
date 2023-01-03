clear all;
close all;
%control coursework - model predictive control
%plots the 4 required plots for B1 - just run the code
%B1
A = [1 0 0; 1 1 0; 0 1 1];
B = [1 0 0]';
C = [0 0 1];

%calculation of Qf
Q = C'*C;

%cost parameter set to 1
roh = 1;
R = roh;

%initial state
x0 = [0.5428 0.7633 0.3504]';

%initalisation of framework to store 50 states of P
P_output = zeros(150,3);
P_output(148:150, 1:3) = Q;
%initialisation of framework to store the state, input and control coef
X_output = zeros(3,48);
U_input = zeros(49,1);
K_output = zeros(49,3);

for i = 49:-1:1
    %discrete ARE but instead of Q replace with an iteration up the P
    %matrix
    P_prev = P_output((3*i-2)+3:3*i+3, 1:3);
    P_output((3*i-2):3*i, 1:3) = Q + (A'*P_prev*A)-(A'*P_prev*B*((R+B'*P_prev*B)^(-1))*B'*P_prev*A);

end
%set the initial X state
X_state = x0;
X_output(:,1) = x0;
%iterate through the initial and other 49 states using the P matrix
for i = 1:49

    P = P_output((3*i-2):3*i, 1:3);
    K = -((R+B'*P*B)^(-1))*B'*P*A;
    K_output(i, :) = K;
    U_optimal = K*X_state;
    X_next = A*X_state + B*U_optimal;
    X_output(:, i+1) = X_next;
    U_input(i) = U_optimal;
    
    %Xt+1 = Xt
    X_state = X_next;

end

%plotting
t = 1:49;
figure(1);
plot(t, U_input)
xlabel('Time (s)')
ylabel('Optimal Input U')

t = 1:50;
figure(2);
hold on;
plot(t, X_output(1,:))
plot(t, X_output(2,:))
plot(t, X_output(3,:))
hold off;
xlabel('Time (s)')
ylabel('State Features in X')

%LQR cost
cost = x0'*P_output(1:3, 1:3)*x0

max(abs(U_input))