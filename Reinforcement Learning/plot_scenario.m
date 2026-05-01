function plot_scenario(nome_scenari, num)

    scenari = load(nome_scenari);

    scenari = scenari.scenari(num);

    waypoints = scenari.sim_pos_des.Data;
    q_start = scenari.map.q_start; 
    q_goal = scenari.map.q_goal; 
    v = scenari.map.v; 
    n_collision = scenari.map.n_collision;

    figure(1); hold on;
    title('Scenario');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    disp('Disegno della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end

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

    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'Color', 'g', 'LineWidth', 4, 'HandleVisibility', 'off');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    
    plot3(q_start(1), q_start(2), q_start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(q_goal(1), q_goal(2), q_goal(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Target');

    legend('Location', 'best');
    
    view(30,30); axis equal; grid on; hold off;
end