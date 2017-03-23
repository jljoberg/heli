close all;

load('sim_out_travel_FB.mat');
load('sim_out_e_FB.mat');
load('sim_u1_FB.mat')
load('sim_u2_FB.mat')

%% Plot

% Plot Elevation
subplot(2,1,1); hold on; grid on; xlim([0 25]);
hx5Sim = plot(sim_out_e(1,:), sim_out_e(2,:));
hx5Ref = plot(xTime, x5*180/pi);
legend('x5 - Elevation meassured', 'x5_{ref} - Elevation ref'); title('Elevation, nonLin with feedback');
xlabel('Time [s]'); ylabel('Angle [deg]');
set(hx5Sim, 'Color', 'g'); set(hx5Sim, 'LineWidth', 1.5);
set(hx5Ref, 'Color', 'r'); set(hx5Ref, 'LineStyle', '--'); set(hx5Ref, 'LineWidth', 1.5);



% Plot travell Ref
subplot(2,1,2); hold on; grid on; xlim([0 25]);
hx1Sim = plot(sim_out_travel(1,:), sim_out_travel(2,:));
hx1Ref = plot(xTime, x1*180/pi);
legend('x1 - Travel meassured', 'x1_{ref} - Travel ref'); title('Travel, nonLin with feedback');
xlabel('Time [s]'); ylabel('Angle [deg]');

set(hx1Sim, 'Color', 'g'); set(hx1Sim, 'LineWidth', 1.5);
set(hx1Ref, 'Color', 'r'); set(hx1Ref, 'LineStyle', '--'); set(hx1Ref, 'LineWidth', 1.5);


%% Save to vector file
%print -depsc plot_nonLin_FB