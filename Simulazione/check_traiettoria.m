% Apriamo una nuova finestra per il grafico
figure('Name', 'Verifica Volo Minimum Snap', 'Color', 'w');

% 1. Disegniamo la traiettoria EFFETTIVA del drone (in blu)
plot3(x, y, z, 'b-', 'LineWidth', 2);
hold on; grid on;

% 2. Disegniamo la traiettoria DESIDERATA (Minimum Snap) (in rosso tratteggiato)
% (Se sim_pos_des è una timeseries, estraiamo i dati, altrimenti li plottiamo diretti)
if isa(sim_pos_des, 'timeseries')
    plot3(sim_pos_des.Data(:,1), sim_pos_des.Data(:,2), sim_pos_des.Data(:,3), 'r--', 'LineWidth', 1.5);
else
    plot3(sim_pos_des(:,1), sim_pos_des(:,2), sim_pos_des(:,3), 'r--', 'LineWidth', 1.5);
end

% Aggiungiamo etichette e leggenda per capire cosa stiamo guardando
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Confronto Traiettoria: Reale vs Minimum Snap');
legend('Volo Reale Drone', 'Riferimento (Minimum Snap)', 'Location', 'best');
view(3); % Imposta una bella vista 3D isometrica