
close all;

%% Load var
load('sim_u.mat');
load('sim_out.mat');

sim_t = sim_out(1,:);

%% Plot
figure
subplot(2,1,1); hold on; grid on;
plot(uTime, u)
plot(sim_t, sim_u(2,:),'r');
legend('Optimal u', 'meassured u')


subplot(2,1,2); hold on; grid on;
plot(xTime, x1*180/pi);
plot(sim_t, sim_out(2,:), 'r');
legend('optimal lambda', 'meassured lambda');
