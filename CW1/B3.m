clear all;
close all;
%control coursework - linear quadratic regulator
%combination of B1 and B2
%B1
A = [1 0 0; 1 1 0; 0 1 1];
B = [1 0 0]';
C = [0 0 1];
Q = C'*C;

%CHANGE ROH
roh = 0.1;
R = roh;

x0 = [0.5428 0.7633 0.3504]';
P_output = zeros(150,3);
P_output(148:150, 1:3) = Q;
X_output = zeros(3,48);
U_input = zeros(49,1);
K_output = zeros(49,3);
for i = 49:-1:1
    P_prev = P_output((3*i-2)+3:3*i+3, 1:3);
    P_output((3*i-2):3*i, 1:3) = Q + (A'*P_prev*A)-(A'*P_prev*B*((R+B'*P_prev*B)^(-1))*B'*P_prev*A);

end
X_state = x0;
X_output(:,1) = x0;
for i = 1:49
    P = P_output((3*i-2):3*i, 1:3);
    K = -((R+B'*P*B)^(-1))*B'*P*A;
    K_output(i, :) = K;
    U_optimal = K*X_state;
    X_next = A*X_state + B*U_optimal;
    X_output(:, i+1) = X_next;
    U_input(i) = U_optimal;
    X_state = X_next;
end

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

figure(3);
hold on;
plot(t, P_output(1:3:end,1))
plot(t, P_output(2:3:end,1))
plot(t, P_output(3:3:end,1))
plot(t, P_output(1:3:end,2))
plot(t, P_output(2:3:end,2))
plot(t, P_output(3:3:end,2))
plot(t, P_output(1:3:end,3))
plot(t, P_output(2:3:end,3))
plot(t, P_output(3:3:end,3))
hold off;
xlabel('Time (s)')
ylabel('Elements in matrix Pt')

t = 1:49;
figure(4);
hold on;
plot(t, K_output(:,1))
plot(t, K_output(:,2))
plot(t, K_output(:,3))
hold off;
xlabel('Time (s)')
ylabel('Values of K')

%LQR cost
cost = x0'*P_output(1:3, 1:3)*x0

%B2
[X,K,L, INFO] = idare(A,B,Q,R);
X_output = zeros(3,49);
X_state = x0;
X_output(:,1) = x0;
U_input = zeros(48,1);

for i = 1:48
    U_optimal = -K*X_state;
    X_next = A*X_state + B*U_optimal;
    X_output(:, i+1) = X_next;
    U_input(i) = U_optimal;
    X_state = X_next;
end

t = 1:48;
figure(5);
plot(t, U_input)

xlabel('Time (s)')
ylabel('Optimal Input U')

t = 1:49;
figure(6);
hold on;
plot(t, X_output(1,:))
plot(t, X_output(2,:))
plot(t, X_output(3,:))
hold off;

xlabel('Time (s)')
ylabel('State Features in X')

%LQR Cost
cost = x0'*X*x0