function graphic_func(log_posizione, idx_scenario, path_DB_scenari)
    disp('Estrazione dati di volo dalla telemetria...');
    
    % log_posizione sarà una matrice. Se il volo dura 500 step, 
    % potrebbe essere [500 x 3] oppure [3 x 500]. Lo rendiamo standard:
    if size(log_posizione, 2) == 3
        X = log_posizione(:, 1);
        Y = log_posizione(:, 2);
        Z = log_posizione(:, 3);
    else
        X = log_posizione(1, :);
        Y = log_posizione(2, :);
        Z = log_posizione(3, :);
    end
    
    % Setup della Figura
    figure('Name', 'Replay Volo SAC', 'Color', 'w', 'Position', [100, 100, 800, 600]);
    hold on; grid on; view(30, 30);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Traiettoria 3D del Drone');

    data = load(path_DB_scenari); 
    scenario = data.scenari(idx_scenario);
    
    v = scenario.map.v;
    pos_des = scenario.sim_pos_des.Data;
    n_collision = scenario.map.n_collision;
    goal = scenario.map.q_goal;
    
    %r = 5.0;
    %num_segments = size(waypts, 2) - 1;

        % ---------------------------------------------------------------------
    % DISEGNO DELLA CITTÀ (Grattacieli Blu)
    % ---------------------------------------------------------------------
    disp('Rendering della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    % % Disegna i chiodi (waypoint) usati dal solver
    % plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'y');
    
    % % ---------------------------------------------------------------------
    % % RENDERING DEL TUBO CONTINUO
    % % ---------------------------------------------------------------------
    % disp('Rendering del Safe Flight Corridor 3D continuo...');
    % for i = 1:num_segments
    %     p1 = waypts(:,i);
    %     p2 = waypts(:,i+1);
    %     dist = norm(p2 - p1);
    %     % Calcola quanti cubi servono per fare un tubo senza buchi
    %     num_cubes = ceil(dist / (r * 1.5)) + 1; 
    %     for j = 0:max(1, num_cubes-1)
    %         cube_center = p1 + (j/max(1, num_cubes-1)) * (p2 - p1);
    %         plot_cube(cube_center, r);
    %     end
    % end
    % 
    % Disegna la traiettoria desiderata 
    plot3(pos_des(:,1), pos_des(:,2), pos_des(:,3), 'Color', [0 0.8 1], 'LineWidth', 1, 'DisplayName', 'Traiettoria');
    
    % % Avvia la vista corretta
    % view(30, 30); grid on; axis equal; hold off;
    
    % Disegna Start e End
    plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End/Crash');

    plot3(goal(1), goal(2), goal(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'Target');
    
    axis([min(X)-5, max(X)+5, min(Y)-5, max(Y)+5, min(Z)-2, max(Z)+5]);

    %plot3(X(:), Y(:), Z(:), 'Color', 'b', 'LineWidth', 2);
    
    % Animazione
    curve = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Ground truth');
    %legend('Location', 'best');

    step_salto = max(1, floor(length(X) / 200)); 
    for i = 1:step_salto:length(X)
        addpoints(curve, X(i), Y(i), Z(i));
        drawnow; 
        pause(0.001); 
    end
    addpoints(curve, X(end), Y(end), Z(end));
    drawnow;
    disp('Replay concluso!');
    pause(2);
    axis equal;
    drawnow;
end