
close all;

%% Load var
load('sim_u.mat');
load('sim_out.mat');

sim_t = sim_out(1,:)

%%Plot
hold on; grid on;
plot(sim_t, 30*sim_u(2,:),'r'); plot(sim_t, sim_out(2,:));
legend('Input (30*u)', 'Output (lambda)');