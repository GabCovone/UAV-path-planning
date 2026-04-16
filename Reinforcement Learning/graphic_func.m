function graphic_func(log_posizione, idx_scenario, path_DB_scenari)
    disp('Estrazione dati di volo dalla telemetria...');
    
    % log_posizione sarà una matrice. Se il volo dura 500 step, 
    % potrebbe essere [500 x 3] oppure [3 x 500]. Lo rendiamo standard in vettori colonna:
    if size(log_posizione, 2) == 3
        X = log_posizione(:, 1);
        Y = log_posizione(:, 2);
        Z = log_posizione(:, 3);
    else
        % Aggiunti gli apici (') per trasporre i vettori riga in vettori colonna
        X = log_posizione(1, :)';
        Y = log_posizione(2, :)';
        Z = log_posizione(3, :)';
    end
    
    % Setup della Figura
    figure('Name', 'Replay Volo SAC', 'Color', 'w', 'Position', [100, 100, 800, 600]);
    hold on; grid on; view(30, 30);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Traiettoria 3D del Drone');

    % Caricamento dati dello scenario
    data = load(path_DB_scenari); 
    scenario = data.scenari(idx_scenario);
    
    v = scenario.map.v;
    pos_des = scenario.sim_pos_des.Data;
    n_collision = scenario.map.n_collision;
    goal = scenario.map.q_goal;
    
    % ---------------------------------------------------------------------
    % DISEGNO DELLA CITTÀ (Grattacieli Blu)
    % ---------------------------------------------------------------------
    disp('Rendering della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
    
    % Disegna la traiettoria desiderata 
    plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), 'Color', [0 0.8 1], 'LineWidth', 1, 'DisplayName', 'Traiettoria Ideale');
    
    % Disegna Start e End/Crash
    plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End/Crash');

    % Disegna il Target/Goal
    plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Target');
    
    % Configurazione degli assi (bounding box)
    axis([min(X)-5, max(X)+5, min(Y)-5, max(Y)+5, min(Z)-2, max(Z)+5]);
    
    % [FIX] Spostato QUI: fondamentale per mantenere le proporzioni reali (cubiche) 
    % della città ed evitare distorsioni visive fin dal primo frame dell'animazione
    axis equal; 
    
    % Animazione
    curve = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Ground truth');
    legend('Location', 'best'); % Riattivata per mostrare le etichette

    step_salto = max(1, floor(length(X) / 200)); 
    for i = 1:step_salto:length(X)
        addpoints(curve, X(i), Y(i), Z(i));
        drawnow; 
        pause(0.001); 
    end
    
    % Assicura che l'ultimo punto venga disegnato per chiudere la traiettoria
    addpoints(curve, X(end), Y(end), Z(end));
    drawnow;
    
    disp('Replay concluso!');
    % (Rimosso il pause(2) e axis equal finale ridondanti)
end