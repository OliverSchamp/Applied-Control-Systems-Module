clear all;
close all;
%sampling time
ts = 0.01;
%reference generation
t0 = 0:ts:1-ts;
t1 = 1:ts:2-ts;
traj1 = 0.035*t0;
traj2 = 0.07-0.035*t1;
rx = cat(2,traj1,traj2);
%tracking
load('x-axis_7th_order_model.mat');
zpk_ = c2d(sysc,0.01);
z = cell2mat(zpk_.Z);
p = cell2mat(zpk_.P);
k = zpk_.K;
[A,B,C,D] = zp2ss(z,p,k);

G = zeros([200, 200]);

for j = 0:200-1
for i = 1:200-j
    G(i+j,i) = C*(A^(j))*B;
end
end

%simulation
for omega = logspace(-16,-6,10)
%omega = 1e-7;
U_inputs = zeros(200*100+200,1);
Y_outputs = zeros(200*100+200,1);
Error_norms = zeros(100,1);
gammas = zeros(100,1);
iterations = 1:100;

for i = 1:100
    idx = (i-1)*200 + 1;
    idx_next = i*200 + 1;
    U_k = U_inputs(idx:idx+199);
    Y_outputs(idx:idx+199) = G*U_k;

    e_k = rx' - G*U_k;
    Error_norms(i) = norm(e_k);
    gamma_num = 2*e_k'*(G*G')*e_k;
    gamma_den = 2*omega + 2*e_k'*((G*G')^2)*e_k;
    gamma = gamma_num/gamma_den;
    gammas(i) = gamma;
    
    U_kk = U_k+gamma*G'*e_k;
    U_inputs(idx_next:idx_next+199) = U_kk;
    
    Y_outputs(idx_next:idx_next+199) = G*U_kk;

end

hold on;
figure(1);
plot(iterations, Error_norms);
xlabel('Iterations');
ylabel('log||error||');

hold on;
figure(2);
plot(iterations, gammas);
end

figure(1);
legend(string(logspace(-16,-6,10)));
set(gca, 'YScale', 'log');
xlabel('Iterations');
ylabel('log||error||');

figure(2);
legend(string(logspace(-16,-6,10)));
xlabel('Iterations');
ylabel('gamma');