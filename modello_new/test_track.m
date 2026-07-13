% =========================================================================
% SCRIPT DI TEST: GENERAZIONE TRAIETTORIA, SIMULAZIONE E GRAFICA
% =========================================================================
clear; clc; close all;

%% 1. CARICAMENTO PARAMETRI E PID (Dal Workspace a Simulink)
% =========================================================================
% PARAMETRI DRONE E MATRICE MIXER AERODINAMICA
% =========================================================================
clear; clc;

% --- FISICA BASE ---
Drone.Mass = 1.0;                            
Drone.Inertia = diag([0.02, 0.02, 0.04]);    
Drone.g = 9.81;                              
Drone.L = 0.225;                             % Braccio: 22.5 cm

% --- PARAMETRI ELICHE (Drone 1kg) ---
Drone.Radius = 0.127;                        % Raggio elica: 12.7 cm (5 pollici)
Drone.Ct = 1.14e-4;                          % Coeff. Spinta base
Drone.Cq = 1.36e-6;                          % Coeff. Coppia base
rho = 1.225;                                 % Densità aria

% --- CALCOLO COEFFICIENTI EQUIVALENTI PER SIMULINK ---
% Questa è la chiave! Simulink usa R^4 per la spinta e R^5 per la coppia
Ct_eq = Drone.Ct * rho * pi * (Drone.Radius^4);
Cq_eq = Drone.Cq * rho * pi * (Drone.Radius^5);

% --- MATRICE DI MIXER COMPLETA ---
MixerMatrix = [ 1/(4*Ct_eq), -1/(4*Ct_eq*Drone.L), -1/(4*Ct_eq*Drone.L),  1/(4*Cq_eq);
                1/(4*Ct_eq),  1/(4*Ct_eq*Drone.L), -1/(4*Ct_eq*Drone.L), -1/(4*Cq_eq);
                1/(4*Ct_eq),  1/(4*Ct_eq*Drone.L),  1/(4*Ct_eq*Drone.L),  1/(4*Cq_eq);
                1/(4*Ct_eq), -1/(4*Ct_eq*Drone.L),  1/(4*Ct_eq*Drone.L), -1/(4*Cq_eq) ];

% --- GUADAGNI PID ---
PID.Z_Kp = 5.0;    PID.Z_Ki = 0.5;   PID.Z_Kd = 2.0;
PID.XY_Kp = 0.5;   PID.XY_Ki = 0.0;  PID.XY_Kd = 0.1;
PID.Att_Kp = 2.0;  PID.Att_Ki = 0.0; PID.Att_Kd = 0.5;
PID.Yaw_Kp = 1.0;  PID.Yaw_Ki = 0.0; PID.Yaw_Kd = 0.2;


%% 2. GENERAZIONE TRAIETTORIA DI RIFERIMENTO (A Spirale)
T_final = 20; % Durata simulazione in secondi
dt = 0.01;
t = (0:dt:T_final)';

r = 5;          % Raggio della spirale (m)
omega = 0.5;    % Velocità angolare di rotazione (rad/s)

% Posizioni ideali di riferimento (Integration check)
x_ref = r * cos(omega * t);
y_ref = r * sin(omega * t);
z_ref = 1 + 0.4 * t; % Il drone sale costantemente

% VELOCITÀ DESIDERATE (Derivate delle posizioni)
vx_des = -r * omega * sin(omega * t);
vy_des = r * omega * cos(omega * t);
vz_des = 0.4 * ones(size(t));

% YAW DESIDERATO (Il drone si gira guardando la direzione del moto)
yaw_des = omega * t + pi/2; 

% Formattazione delle strutture per i blocchi "From Workspace"
vel_des_signal.time = t;
vel_des_signal.signals.values = [vx_des, vy_des, vz_des];
vel_des_signal.signals.dimensions = 3;

yaw_des_signal.time = t;
yaw_des_signal.signals.values = yaw_des;
yaw_des_signal.signals.dimensions = 1;

%% 3. ESECUZIONE DELLA SIMULAZIONE IN SIMULINK
disp('Avvio della simulazione in Simulink...');
% Esegue il modello. Sostituisci 'modello_drone' con il nome esatto del tuo file .slx
simOut = sim('modello', 'StopTime', num2str(T_final)); 
disp('Simulazione completata con successo.');

%% 4. ESTRAZIONE DATI DI OUTPUT
t_sim = simOut.tout;

% Estrazione diretta dai blocchi To Workspace (formato Array)
try
    pos_reale = simOut.pos_reale; % Prende la matrice [N x 3] dal Workspace di output
    vel_reale = simOut.vel_reale; % Prende la matrice [N x 3] dal Workspace di output
catch
    error('Assicurati che i blocchi To Workspace si chiamino "pos_reale" e "vel_reale" e siano impostati su "Array".');
end

%% 5. GRAFICI DEI RISULTATI
figure('Name', 'Verifica Tracking Traiettoria Drone', 'Position', [100, 100, 1200, 600]);

% --- GRAFICO 1: Spazio 3D ---
subplot(1, 2, 1);
plot3(x_ref, y_ref, z_ref, 'r--', 'LineWidth', 1.5); hold on; grid on;
plot3(pos_reale(:,1), pos_reale(:,2), pos_reale(:,3), 'b-', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Traiettoria 3D del Drone');
legend('Riferimento Teorico', 'Risposta Reale Drone', 'Location', 'Best');
view(3);

% --- GRAFICO 2: Inseguimento Velocità (Assi X, Y, Z) ---
subplot(1, 2, 2);
plot(t, vx_des, 'r--', 'LineWidth', 1); hold on; grid on;
plot(t_sim, vel_reale(:,1), 'b-', 'LineWidth', 1.5);
plot(t, vy_des, 'g--', 'LineWidth', 1);
plot(t_sim, vel_reale(:,2), 'm-', 'LineWidth', 1.5);
plot(t, vz_des, 'k--', 'LineWidth', 1);
plot(t_sim, vel_reale(:,3), 'c-', 'LineWidth', 1.5);

xlabel('Tempo (s)'); ylabel('Velocità (m/s)');
title('Confronto Velocità Desiderata vs Reale');
legend('Vx Desiderata', 'Vx Reale', 'Vy Desiderata', 'Vy Reale', 'Vz Desiderata', 'Vz Reale', 'Location', 'Best');