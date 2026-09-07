%% Actividad Filtros

clear; clc; close all;
rng(40);
%% Definir parametros
dt =0.1;
T =10;
N= T/dt;

v = 0.5;
w = 0.3;

x0= [0; 0; 0];

%% Trayectoría realizada
x_r = zeros(3,N); %x,y,theta
x_r(:,1)= x0;
for k=2:N
    x_r(:,k) = movimiento(x_r(:,k-1), v, w, dt);
end

% Visualizar la trayectoria
plot(x_r(1,:), x_r(2,:), 'b-');
xlabel('X');
ylabel('Y');
title('Trayectoria real del robot');
grid on;
axis equal;
%% Simulacion de sensores y ruido

%ruido
sigma_gps= 0.12; %m
sigma_imu = deg2rad(4); %rad

%mediciones
z_gps = zeros(2,N); %x,y
z_imu = zeros(1,N); %theta

for k = 1:N 
    z_gps(:,k) = x_r(1:2,k) + sigma_gps * randn(2,1); % GPS measurement with noise
    z_imu(k) = x_r(3,k) + sigma_imu * randn(1,1); % IMU measurement with noise
end

%%  Visualizar las mediciones
figure;
%GPS
plot(x_r(1,:),x_r(2,:), 'b-');
hold on;
plot(z_gps(1,:), z_gps(2,:), 'ro'); % Plot GPS 
title('Trayectoria real vs Mediciones GPS');
legend('Trayectoria real', 'Mediciones GPS');
xlabel('x[m]');
ylabel('y[m]');
hold off;

% IMU
t = (0:N-1)*dt;
% Visualizar las mediciones IMU
figure;
plot(t, x_r(3,:), 'b-');
hold on;
plot(t,z_imu, 'g.'); % Plot IMU measurements
grid on;
xlabel('t(s)');
ylabel('theta (rad)')
title('Angulo z real vs mediciones IMU')
legend('z real', 'Mediciones IMU');
%% Odometria para trayectoria con ruido

ruido_v = 0.08;
ruido_w =deg2rad(2);

v_odo = v + ruido_v * randn(1,N);
w_odo = w + ruido_w * randn(1, N);

%% EKF

% Incertidumbre mov
Q = diag([ruido_v * dt, ruido_v *dt, ruido_w*dt]).^2;

%Incertidumbre sensores
R = diag([sigma_gps, sigma_gps, sigma_imu]).^2; %ruido en sensores

sigma_inicial= [0.05; 0.05; 0.05];
P0 = diag(sigma_inicial.^2); %confianza inicial

[x_estim, P_h] = eKf_l(x0, P0, v_odo, w_odo, dt, z_gps,z_imu,Q,R);

% Visualizar la estimación
figure;
plot(x_r(1,:), x_r(2,:), 'b-'); % Trayectoria real
hold on;
plot(z_gps(1,:), z_gps(2,:), 'r.')
plot(x_estim(1,:), x_estim(2,:), ':m'); % Estimación del estado
title('Trayectoria real vs Estimación del estado');
legend('Trayectoria real', 'Gps con ruido', 'EKF');
xlabel('x[m]');
ylabel('y[m]');
grid on;
axis equal;
hold off;
%% UKF

[x_estim_ukf, P_h_ukf]= ukf_l(x0,P0, v_odo,w_odo,dt,z_gps, z_imu,Q,R);

% Visualizar la estimación con UKF
figure;
plot(x_r(1,:), x_r(2,:), 'b-'); % Trayectoria real
hold on;
plot(z_gps(1,:), z_gps(2,:), 'r.'); % Mediciones GPS
plot(x_estim_ukf(1,:), x_estim_ukf(2,:), 'g--'); % Estimación del estado con UKF
title('Trayectoria real vs Estimación del estado (UKF)');
legend('Trayectoria real', 'Gps con ruido', 'UKF');
xlabel('x[m]');
ylabel('y[m]');
grid on;
axis equal;
hold off;
%% MCL

M = 800;
[x_mcl, particulas_h] = mcl_l(x0,M, P0, v_odo,w_odo, dt, z_gps, z_imu, Q, R);

% Visualizar la estimación con MCL
figure;
plot(x_r(1,:), x_r(2,:), 'b-'); % Trayectoria real
hold on;
plot(z_gps(1,:), z_gps(2,:), 'r.'); % Mediciones GPS
plot(x_mcl(1,:), x_mcl(2,:), 'k--'); % Estimación del estado con MCL
title('Trayectoria real vs Estimación del estado (MCL)');
legend('Trayectoria real', 'Gps con ruido', 'MCL');
xlabel('x[m]');
ylabel('y[m]');
grid on;
axis equal;
hold off;
%% Evolucion temporal del estado

figure;

%para x
subplot(3,1,1);
plot(t, x_r(1,:), 'b', 'LineWidth', 0.5);
hold on;
plot(t, x_estim(1,:), '--r');
plot(t, x_estim_ukf(1,:), '-.g');
plot(t, x_mcl(1,:), ':k');
ylabel('x[m]');
title('Evolucion temporal del estado');
legend('Real', 'EKF','UKF', 'MCL');
grid on;

%para y
subplot(3,1,2);
plot(t,x_r(2,:), 'b', 'LineWidth', 0.5);
hold on;
plot(t, x_estim(2,:), '--r');
plot(t, x_estim_ukf(2,:), '-.g');
plot(t, x_mcl(2,:), ':k');
ylabel('y[m]');
grid on;

%para theta
subplot(3,1,3);
plot(t, x_r(3,:), 'b', 'LineWidth', 0.5);
hold on;
plot(t, x_estim(3,:), '--r');
plot(t, x_estim_ukf(3,:), '-.g');
plot(t, x_mcl(3,:), '--k');
ylabel('theta [rad]');
xlabel('tiempo [s]')
grid on;

%% Comparacion

% Comparar resultados
fig_comparison = figure('Color', 'w', 'Position', [100 100 900 600]);


plot(x_r(1,:), x_r(2,:), 'k-', 'Linewidth',2); 
hold on;
plot(z_gps(1,:), z_gps(2,:), '.', 'Color', [0.65 0.65 0.65], 'MarkerSize', 8); % Mediciones GPS
plot(x_mcl(1,:), x_mcl(2,:), '--', 'Color', [0.4660 0.6740 0.1880], 'LineWidth', 1.4); % Estimación del estado con MCL
plot(x_estim(1,:), x_estim(2,:), '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.4); % Estimación del estado con ekf
plot(x_estim_ukf(1,:), x_estim_ukf(2,:), '-.', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.4); % Estimación del estado con UKF

title('Localization Filter Comparison');

legend('Ground truth', 'Noisy GPS', 'MCL', 'EKF', 'UKF', 'Location', 'best');

xlabel('x[m]');
ylabel('y[m]');
grid on;
axis equal;
set(gca, 'FontSize', 11);
hold off;

% Save the figure used in the README
script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

exportgraphics( fig_comparison, fullfile(results_dir, 'localization_comparison.png'), 'Resolution', 180);

%% Metricas
%% Raiz del error cuadratico medio RMSE


error_ekf = sqrt(sum((x_r(1:2,:) - x_estim(1:2,:)).^2,1));
rmse_ekf = sqrt(mean(error_ekf.^2));
max_error_ekf = max(error_ekf);
std_error_ekf = std(error_ekf);


error_ukf = sqrt(sum((x_r(1:2,:) - x_estim_ukf(1:2,:)).^2, 1));
rmse_ukf = sqrt(mean(error_ukf.^2));
max_error_ukf = max(error_ukf);
std_error_ukf = std(error_ukf);

error_mcl = sqrt(sum((x_r(1:2,:) - x_mcl(1:2,:)).^2, 1));
rmse_mcl = sqrt(mean(error_mcl.^2));
max_error_mcl = max(error_mcl);
std_error_mcl = std(error_mcl);

%Error angular
error_theta_ekf =atan2( sin(x_r(3,:)- x_estim(3,:)), cos(x_r(3,:) - x_estim(3,:)));
error_theta_ukf =atan2( sin(x_r(3,:)- x_estim_ukf(3,:)), cos(x_r(3,:) - x_estim_ukf(3,:)));
error_theta_mcl =atan2( sin(x_r(3,:)- x_mcl(3,:)), cos(x_r(3,:) - x_mcl(3,:)));


rmse_theta_ekf = rad2deg(sqrt(mean(error_theta_ekf.^2)));
rmse_theta_ukf = rad2deg(sqrt(mean(error_theta_ukf.^2)));
rmse_theta_mcl = rad2deg(sqrt(mean(error_theta_mcl.^2)));

%mostrar resultados 

fprintf('\n--- RMSE de posicion ---\n');
fprintf('EKF: %.4f m | UKF: %.4f m | MCL: %.4f m\n', rmse_ekf, rmse_ukf, rmse_mcl);

fprintf('\n--- RMSE angular ---\n');
fprintf('EKF: %.2f deg | UKF: %.2f deg | MCL: %.2f deg\n', rmse_theta_ekf, rmse_theta_ukf, rmse_theta_mcl);

fprintf('\n--- Error Maximo de posicion ---\n');
fprintf('EKF: %.4f m | UKF: %.4f m | MCL: %.4f m\n', max_error_ekf, max_error_ukf, max_error_mcl);

fprintf('\n--- Desviacion estandar del error de posicion ---\n');
fprintf('EKF: %.4f m | UKF: %.4f m | MCL: %.4f m\n', std_error_ekf,std_error_ukf,std_error_mcl);