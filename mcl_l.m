function [x_estim, particulas_h] = mcl_l(x0,M,P0,v_odmt,w_odmt, dt, z_gps, z_imu, Q,R)
% Localización de Monte Carlo para estimación de estado del robot
% x0: edo. incial del robot [x; y; theta]
% M: número de partículas
% P0: matriz de covarianza inicial
% v_odmt: vel lineal por odometría
% w_odmt: vel angular por odometría
% dt: paso de tiempo de la simulación
% z_gps: mediciones del GPS
% z_imu: mediciones del IMU
% Q: matriz de covarianza del ruido del modelo de movimiento
% R: matriz de covarianza del ruido de los sensores
    N = size(z_gps,2);
    x_estim = zeros(3, N);
    particulas_h = zeros(3,M,N);

    %particulas iniciales en t=0
    L0 = chol(P0,'lower');
    particulas = x0 + L0 * randn(3,M);
    pesos = ones(1,M) /M;

    x_estim(:,1)= mean(particulas,2);
    x_estim(3,1) = atan2(mean(sin(particulas(3,:))), mean(cos(particulas(3,:))));
    particulas_h(:,:,1) = particulas;

    L_R = chol(R,'lower'); %R es constante

    for k =2:N

        %Prediccion
        for i= 1:M
            x_sig = movimiento(particulas(:,i), v_odmt(k-1), w_odmt(k-1), dt);

            particulas(:,i) = x_sig + sqrt(diag(Q)) .* randn(3,1); %ruido
        end

        %Actualizacion
        z_r = [z_gps(1,k); z_gps(2,k); z_imu(k)];

        for i = 1:M
            %error
            y_i = z_r - particulas(:,i);
            y_i(3) = atan2(sin(y_i(3)), cos(y_i(3)));


            residuo_n = L_R \ y_i;
            pesos(i) = exp(-0.5 * (residuo_n' * residuo_n)) + 1e-300; %evitar ceros
        end

        %normalizar pesos
        pesos = pesos / sum(pesos);

        %Resampling

        indices = datasample(1:M, M, 'Weights', pesos, 'Replace', true);
        particulas = particulas(:,indices);

        %reiniciar pesos
        pesos = ones(1,M) / M;

        %edo final
        x_estim(:, k) = mean(particulas, 2); % Estimar la posición
        x_estim(3,k) = atan2(mean(sin(particulas(3,:))), mean(cos(particulas(3,:))));
        particulas_h(:, :, k) = particulas; % Almacenar partículas para análisis posterior

    end
end