function nuevo_edo = movimiento(edo_anterior, v ,w, dt)
% Modelo de movimiento del robot diferencial
% edo_anterior: estado anterior del robot [x; y; theta]
% v: velocidad lineal
% w: velocidad angular
% dt: paso de tiempo
% nuevo_edo: nuevo estado del robot [x; y; theta]

   theta_ant = edo_anterior(3);
   x_t = edo_anterior(1) + v * cos(theta_ant) * dt;
   y_t = edo_anterior(2) + v * sin(theta_ant) * dt;
   theta_t = theta_ant + w * dt;

   nuevo_edo = [x_t; y_t; theta_t];

end