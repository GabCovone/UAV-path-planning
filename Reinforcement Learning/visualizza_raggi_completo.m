function visualizza_raggi_completo(path_DB_scenari, idx_scenario, log_posizione, vettore_tempi, lidar_scan, pos_drone, num_rays)
    set(gcf, 'Renderer', 'opengl');

    % Se lidar_scan e pos_drone sono forniti, visualizza anche i raggi
    show_lidar = nargin >= 5 && ~isempty(lidar_scan) && ~isempty(pos_drone) && ~isempty(num_rays);

    if show_lidar
        disp('Generazione del grafico 3D con raggi LIDAR in corso...');
    else
        disp('Generazione del grafico 3D statico in corso...');
    end

    % 1. Standardizzazione della log_posizione in vettori colonna [N x 1]
    if size(log_posizione, 2) == 3
        X = log_posizione(:, 1);
        Y = log_posizione(:, 2);
        Z = log_posizione(:, 3);
    else
        X = log_posizione(1, :)';
        Y = log_posizione(2, :)';
        Z = log_posizione(3, :)';
    end

    % 2. Recupero del Sample Time (Ts) e Calcolo Tempo Finale
    try
        Ts = evalin('base', 'Ts');
    catch
        Ts = 0.1;
    end

    % Calcoliamo a che istante di tempo finisce la simulazione
    if nargin > 3 && ~isempty(vettore_tempi)
        t_end = vettore_tempi(end);
    else
        t_end = (length(X) - 1) * Ts;
    end

    % 3. Setup della Figura
    set(gcf, 'Name', 'Analisi Volo SAC (Statica + LIDAR)')
    set(gcf, 'Color', 'w');
    set(gcf,'Position', [100, 100, 1000, 800]);
    hold on; grid on; view(30, 30);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    if show_lidar
        title('Traiettoria 3D Completa vs Ostacoli Dinamici con Raggi LIDAR');
    else
        title('Traiettoria 3D Completa vs Ostacoli Dinamici');
    end

    % % 4. Caricamento Dati Scenario
    data = load(path_DB_scenari);
    scenario = data.scenari(idx_scenario);

    v = scenario.map.v;
    pos_des = scenario.sim_pos_des.Data;
    n_collision = scenario.map.n_collision;
    goal = scenario.map.q_goal;
    dyn_obs = scenario.dynamic_obstacles;

    % 5. Calcolo della regione intorno al drone (per limitare la visualizzazione)
    if show_lidar
        % Trova il punto corrente del drone (usiamo l'ultimo punto se non specificato)
        if nargin >= 6 && ~isempty(pos_drone)
            drone_pos = pos_drone(:);
        else
            drone_pos = [X(end); Y(end); Z(end)];
        end

        % Calcola i limiti della regione intorno al drone
        region_size = 20; % Dimensione della regione in metri
        x_lim = [drone_pos(1)-region_size, drone_pos(1)+region_size];
        y_lim = [drone_pos(2)-region_size, drone_pos(2)+region_size];
        z_lim = [drone_pos(3)-region_size, drone_pos(3)+region_size];

        % Aggiusta i limiti per includere anche gli edifici vicini
        for k = 1:n_collision
            V_b = v(:,:,k);
            x_min = min(V_b(:,1)); x_max = max(V_b(:,1));
            y_min = min(V_b(:,2)); y_max = max(V_b(:,2));
            z_min = min(V_b(:,3)); z_max = max(V_b(:,3));

            x_lim(1) = min(x_lim(1), x_min - 2);
            x_lim(2) = max(x_lim(2), x_max + 2);
            y_lim(1) = min(y_lim(1), y_min - 2);
            y_lim(2) = max(y_lim(2), y_max + 2);
            z_lim(1) = min(z_lim(1), z_min - 2);
            z_lim(2) = max(z_lim(2), z_max + 2);
        end
    else
        % Usa i limiti originali se non si visualizzano i raggi
        x_lim = [min(X)-15, max(X)+15];
        y_lim = [min(Y)-15, max(Y)+15];
        z_lim = [min(Z)-5, max(Z)+15];
    end

    % 6. Rendering Città (Ostacoli Statici - OTTIMIZZATO PER LE PERFORMANCE)
    disp('Disegno della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k);
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];

        % Disegna solo se l'edificio è nella regione di interesse
        if show_lidar
            if any(V_b(:,1) >= x_lim(1) & V_b(:,1) <= x_lim(2)) && ...
               any(V_b(:,2) >= y_lim(1) & V_b(:,2) <= y_lim(2)) && ...
               any(V_b(:,3) >= z_lim(1) & V_b(:,3) <= z_lim(2))
                patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], ...
                      'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
            end
        else
            patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], ...
                  'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        end
    end

    % 7. Rendering Ostacoli Dinamici (Traiettorie e Sfere)
    if ~isempty(dyn_obs)
        % Calcolo scala visiva per mappe molto grandi
        max_span = max(max(X)-min(X), max(Y)-min(Y));
        visual_scale = max(1, max_span / 500);
        [sphere_X, sphere_Y, sphere_Z] = sphere(20);

        for k = 1:length(dyn_obs)
            r_visivo = dyn_obs(k).radius * visual_scale;
            p0 = dyn_obs(k).p0;
            p_end = p0 + dyn_obs(k).v * t_end; % Posizione finale dell'ostacolo

            % Disegna Sfera di PARTENZA (Più opaca)
            surf(sphere_X * r_visivo + p0(1), sphere_Y * r_visivo + p0(2), sphere_Z * r_visivo + p0(3), ...
                 'FaceColor', [1 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');

            % Disegna Sfera di ARRIVO (Trasparente, indica la direzione)
            surf(sphere_X * r_visivo + p_end(1), sphere_Y * r_visivo + p_end(2), sphere_Z * r_visivo + p_end(3), ...
                 'FaceColor', [1 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

            % Disegna la LINEA di moto dell'ostacolo
            plot3([p0(1), p_end(1)], [p0(2), p_end(2)], [p0(3), p_end(3)], ...
                  'Color', [1 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1.5, ...
                  'DisplayName', ['Moto Ostacolo ' num2str(k)]);
        end
    end

    % 8. Rendering Elementi Drone ISTANTANEO
    % Traiettoria Ideale
    plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), 'Color', [0 0.8 1], 'LineWidth', 1, 'DisplayName', 'Traiettoria Ideale');

    % Traiettoria reale
    plot3(X, Y, Z, 'Color', 'b', 'LineWidth', 2.5, 'DisplayName', 'Traiettoria reale');

    % Marker Start, End, Target
    %plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End/Crash');
    plot3(goal(1), goal(2), goal(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Target');

    % 9. Rendering Raggi LIDAR (se richiesto)
    if show_lidar
        % Genera le direzioni dei raggi (stessa logica della funzione originale)
        indices = 0:(num_rays - 1);
        phi = pi * (3 - sqrt(5)); % Angolo aureo in radianti

        z = 1 - (indices ./ (num_rays - 1)) .* 2;
        raggio_xy = sqrt(1 - z.^2);
        theta = phi .* indices;

        x = cos(theta) .* raggio_xy;
        y = sin(theta) .* raggio_xy;

        dirs = [x; y; z]; % Matrice 3 x num_rays di vettori direzione unitari

        % Parametri del sensore
        max_range = 20.0; % Raggio massimo in metri

        % Calcola le posizioni finali dei raggi (scalate per le distanze lidar)
        ray_endpoints = drone_pos + dirs .* (lidar_scan * max_range);

        % Disegna i raggi come linee blu semi-trasparenti
        for i = 1:num_rays
            if lidar_scan(:,i) ~= 1
                plot3([drone_pos(1), ray_endpoints(1,i)], ...
                      [drone_pos(2), ray_endpoints(2,i)], ...
                      [drone_pos(3), ray_endpoints(3,i)], 'b-', 'LineWidth', 0.5, 'Color', [0 0 1 0.3]);
            end
        end

        % Disegna il drone come un punto rosso
        plot3(drone_pos(1), drone_pos(2), drone_pos(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Drone');
    end

    % 10. Configurazione Finale Assi e Legenda
    axis([x_lim(1), x_lim(2), y_lim(1), y_lim(2), z_lim(1), z_lim(2)]);
    axis equal;
    legend('Location', 'bestoutside');

    disp('Grafico generato con successo!');
end