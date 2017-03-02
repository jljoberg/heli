close all;

load('sim_out_travel.mat');
load('sim_out_e.mat');
load('sim_u1.mat')
load('sim_u2.mat')

%% Plot

hold on;
plot(sim_out_e(1,:), sim_out_e(2,:));
%plot(sim_out_travel(1,:), sim_out_travel(2,:))