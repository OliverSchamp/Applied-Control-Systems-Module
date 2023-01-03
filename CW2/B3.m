%B3 constrained optimisation, use quadprog
clear all;
close all;
A = [1 0 0; 1 1 0; 0 1 1];
B = [1 0 0]';
C = [0 0 1];

%calculation of Qf
Q = C'*C;

%initial state
x0 = [0.5428 0.7633 0.3504]';

%horizon N=5 MPC

G = zeros([15,5]);
G(4:6,1) = B;
G(7:9,2) = B;
G(10:12,3) = B;
G(13:15,4) = B;
G(7:9,1) = A*B;
G(10:12,2) = A*B;
G(13:15,3) = A*B;
G(10:12,1) = A*A*B;
G(13:15,2) = A*A*B;
G(13:15,1) = A*A*A*B;

H = [eye(3); A; A*A; A*A*A; A*A*A*A];

roh = 1;
R = eye(5)*roh;

Q_new = zeros([15,15]);
for i = 1:5
    start = 3*i-2;
    end_ = 3*i;
    Q_new(start:end_,start:end_) = Q;
end

quadprog_1 = R+G'*Q_new*G;
F = [eye(5);-eye(5)];
b = ones([10,1]);


%{
%calculate K
den = R + G'*Q_new*G;
num = G'*Q_new*H;
K_unfiltered = (den^(-1))*num;
K = -[1 0 0 0 0]*K_unfiltered;
%}

%set up
X_output = zeros(3,50);
U_input = zeros(49,1);
%set the initial X state
X_state = x0;
X_output(:,1) = x0;
final_cost = 0;
%iterate through the initial and other 49 states using the P matrix
for i = 1:49
    %U_optimal = K*X_state;
    %U_input(i) = U_optimal;
    quadprog_2 = X_state'*H'*Q_new*G;
    X = quadprog(quadprog_1, quadprog_2, F, b);
    U_optimal = X(1);
    U_input(i) = U_optimal;

    X_next = A*X_state + B*U_optimal;
    X_output(:, i+1) = X_next;

    cost = (X_state'*X_state) + U_optimal^2;
    final_cost = final_cost + cost;
    
    %Xt+1 = Xt
    X_state = X_next;

end

t = 1:49;
figure(3);
plot(t, U_input)
xlabel('Time (s)')
ylabel('Optimal Input U')

t = 1:50;
figure(4);
hold on;
plot(t, X_output(1,:))
plot(t, X_output(2,:))
plot(t, X_output(3,:))
hold off;
xlabel('Time (s)')
ylabel('State Features in X')

max(abs(U_input))

%cost calculations
final_cost