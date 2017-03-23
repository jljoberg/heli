close all;

%% Load var
load('sim_u.mat');
load('sim_out.mat');

sim_t = sim_out(1,:);

%% Plot
figure
subplot(2,1,1); hold on; grid on; xlim([0 25]);
huRef = plot(uTime, u);
huSim = plot(sim_t, sim_u(2,:),'r');
legend('Optimal u', 'meassured u'); title('Using calculated optimal input: Input - pitch reference');
xlabel('Time [s]'); ylabel('Angle [deg]');
set(huSim, 'Color', 'g'); set(huSim, 'LineWidth', 1.5);
set(huRef, 'Color', 'r'); set(huRef, 'LineStyle', '--'); set(huRef, 'LineWidth', 1.5);


subplot(2,1,2); hold on; grid on; xlim([0 25]);
hx1Ref = plot(x1Time, (x1-pi)*180/pi);
hx1Sim = plot(sim_t, sim_out(2,:), 'r');
legend('optimal lambda', 'meassured lambda'); title('Using calculated optimal input: Travel');
xlabel('Time [s]'); ylabel('Angle [deg]');
set(hx1Sim, 'Color', 'g'); set(hx1Sim, 'LineWidth', 1.5);
set(hx1Ref, 'Color', 'r'); set(hx1Ref, 'LineStyle', '--'); set(hx1Ref, 'LineWidth', 1.5);


%% Print to vector file
print -depsc plotOpenLoopTravel