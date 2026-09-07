function [x_estim,P_h] = eKf_l(x0,P0,v_odmt,w_odmt, dt, z_gps, z_imu, Q,R)
% Filtro de Kalman Extendido para estimación de estado del robot
% x0: edo. incial del robot [x; y; theta]
% P0: matriz de covarianza inicial
% v_odmt: vel lineal por odometría
% w_odmt: vel angular por odometría
% dt: paso de tiempo de la simulación
% z_gps: mediciones del GPS
% z_imu: mediciones del IMU
% Q: matriz de covarianza del ruido del modelo de movimiento
% R: matriz de covarianza del ruido de los sensores

N = size(z_gps,2);
x_estim = zeros(3,N);
P_h = zeros(3,3,N);

x=x0; %edo actual
P=P0; %incertidumbre actual

% El primer edo actual es t= 0
x_estim(:,1)= x;
P_h(:,:,1)=P;

for k =2:N
    %prediccion del edo
    theta= x(3);
    x_pr = movimiento(x, v_odmt(k-1), w_odmt(k-1), dt);

    %Jacobiano del movimiento
    F = [1, 0, -v_odmt(k-1) * sin(theta) * dt;
         0, 1,  v_odmt(k-1) * cos(theta) * dt;
         0, 0, 1];

    P_pred = F * P * F' + Q; % prediccion incetidumbre

    H = eye(3);
    z = [z_gps(1,k); z_gps(2,k); z_imu(k)]; %lectura de los sensores

    y = z - x_pr; %error  ---> innovacion
    y(3) = atan2(sin(y(3)), cos(y(3)));

    %ganancia de kalman K
    S = H*P_pred*H'+ R;
    K = P_pred*H'/S;

    % Actualizar estimación edo e incertidumbre --> Correccion
    x= x_pr + K*y;
    P= (eye(3) - K * H) * P_pred;


    %guardar datos
    x_estim(:,k) = x;
    P_h(:,:,k) = P;
 end


end