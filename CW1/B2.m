clear all;
close all;
%plots the two plots and cost required for B2
%B2
A = [1 0 0; 1 1 0; 0 1 1];
B = [1 0 0]';
C = [0 0 1];

Q = C'*C;

%set to 1
roh = 1;
R = roh;

x0 = [0.5428 0.7633 0.3504]';

%steady state - K is given back as a constant
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

%plotting
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

