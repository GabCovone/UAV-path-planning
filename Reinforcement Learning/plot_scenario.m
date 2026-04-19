function plot_scenario(nome_scenari, num)

    scenari = load(nome_scenari);

    scenari = scenari(num).scenari;

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

    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'Color', 'g', 'LineWidth', 4, 'HandleVisibility', 'off');
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    
    plot3(q_start(1), q_start(2), q_start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(q_goal(1), q_goal(2), q_goal(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Target');

    legend('Location', 'best');
    
    view(30,30); axis equal; grid on; hold off;
end