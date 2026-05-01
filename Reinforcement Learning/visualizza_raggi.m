function visualizza_raggi(pos_drone, lidar_scan)
    % Funzione per visualizzare i raggi lidar in 3D
    %
    % Parametri:
    %   pos_drone: [x, y, z] posizione del drone
    %   lidar_scan: array [1 x num_rays] con le distanze normalizzate dei raggi
    %   num_rays: numero di raggi lidar (opzionale se lidar_scan è già definito)

    % Parametri del sensore
    max_range = 20.0; % Raggio massimo in metri

    num_rays = size(lidar_scan, 2);

    % Genera le direzioni dei raggi (stessa logica della funzione originale)
    indices = 0:(num_rays - 1);
    phi = pi * (3 - sqrt(5)); % Angolo aureo in radianti

    z = 1 - (indices ./ (num_rays - 1)) .* 2;
    raggio_xy = sqrt(1 - z.^2);
    theta = phi .* indices;

    x = cos(theta) .* raggio_xy;
    y = sin(theta) .* raggio_xy;

    dirs = [x; y; z]; % Matrice 3 x num_rays di vettori direzione unitari

    % Calcola le posizioni finali dei raggi (scalate per le distanze lidar)
    ray_endpoints = pos_drone(:) + dirs .* (lidar_scan * max_range);

    % Crea la figura
    figure;
    hold on;
    grid on;
    axis equal;
    title('Visualizzazione raggi LIDAR');
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    view(3); % Vista 3D

    % Disegna il drone come un punto rosso
    plot3(pos_drone(1), pos_drone(2), pos_drone(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(pos_drone(1), pos_drone(2), pos_drone(3), ' Drone', 'VerticalAlignment', 'bottom');

    % Disegna i raggi come linee blu
    for i = 1:num_rays
        if lidar_scan(:,i) ~= 1
            plot3([pos_drone(1), ray_endpoints(1,i)], ...
                  [pos_drone(2), ray_endpoints(2,i)], ...
                  [pos_drone(3), ray_endpoints(3,i)], 'b-', 'LineWidth', 1);
        end
    end

    % Imposta i limiti degli assi per includere il drone e i raggi
    all_points = [pos_drone(:), ray_endpoints];
    axis_min = min(all_points, [], 2) - 1;
    axis_max = max(all_points, [], 2) + 1;
    axis([axis_min(1), axis_max(1), axis_min(2), axis_max(2), axis_min(3), axis_max(3)]);

    % Aggiungi una griglia per migliorare la visualizzazione 3D
    set(gca, 'GridAlpha', 0.3, 'MinorGridAlpha', 0.1);

    hold off;
end