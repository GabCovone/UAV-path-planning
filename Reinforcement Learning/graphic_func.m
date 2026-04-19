function graphic_func(path_DB_scenari, idx_scenario, log_posizione, vettore_tempi)
    set(gcf, 'Renderer', 'opengl');

    disp('Generazione del grafico 3D statico in corso...');
    
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
    set(gcf, 'Name', 'Analisi Volo SAC (Statica)')
    set(gcf, 'Color', 'w');
    set(gcf,'Position', [100, 100, 800, 600]);
    hold on; grid on; view(30, 30);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Traiettoria 3D Completa vs Ostacoli Dinamici');
    
    % % 4. Caricamento Dati Scenario
    data = load(path_DB_scenari); 
    scenario = data.scenari(idx_scenario);

    v = scenario.map.v;
    pos_des = scenario.sim_pos_des.Data;
    n_collision = scenario.map.n_collision;
    goal = scenario.map.q_goal;
    dyn_obs = scenario.dynamic_obstacles;
    
    % 5. Rendering Città (Ostacoli Statici - OTTIMIZZATO PER LE PERFORMANCE)
    % disp('Rendering della città 3D in corso...');

    disp('Disegno della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    
    % % Pre-allochiamo le matrici per massima velocità
    % num_faces_per_block = 6;
    % num_vertices_per_block = 8;
    % 
    % F_tot = zeros(n_collision * num_faces_per_block, 4);
    % V_tot = zeros(n_collision * num_vertices_per_block, 3);
    % 
    % base_f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    % 
    % for k = 1:n_collision
    %     V_b = v(:,:,k); 
    % 
    %     % Calcoliamo l'offset per non sovrascrivere i vertici precedenti
    %     v_offset = (k - 1) * num_vertices_per_block;
    % 
    %     % Inseriamo i vertici
    %     V_tot(v_offset + 1 : v_offset + num_vertices_per_block, :) = V_b;
    % 
    %     % Inseriamo le facce, "scalandole" con l'offset dei vertici
    %     f_idx = (k - 1) * num_faces_per_block + 1 : k * num_faces_per_block;
    %     F_tot(f_idx, :) = base_f + v_offset;
    % end
    
    % % Disegniamo l'INTERA CITTÀ con un singolo comando!
    % patch('Faces', F_tot, 'Vertices', V_tot, 'FaceColor', [0.2 0.5 0.8], ...
    %       'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % 6. Rendering Ostacoli Dinamici (Traiettorie e Sfere)
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
    
    % 7. Rendering Elementi Drone ISTANTANEO
    % Traiettoria Ideale
    plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), 'Color', [0 0.8 1], 'LineWidth', 1, 'DisplayName', 'Traiettoria Ideale');
    
    % Traiettoria reale
    plot3(X, Y, Z, 'Color', 'b', 'LineWidth', 2.5, 'DisplayName', 'Traiettoria reale');
    
    % Marker Start, End, Target
    plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End/Crash');
    plot3(goal(1), goal(2), goal(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Target');
    
    % 8. Configurazione Finale Assi e Legenda
    axis([min(X)-15, max(X)+15, min(Y)-15, max(Y)+15, min(Z)-5, max(Z)+15]);
    axis equal; 
    legend('Location', 'best');
    
    disp('Grafico generato con successo!');
end