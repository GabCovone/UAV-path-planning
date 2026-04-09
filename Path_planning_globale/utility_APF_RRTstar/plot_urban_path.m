function plot_urban_path(waypoints_raw, waypoints_pruned, q_start, q_goal, v, n_collision)
    figure(1); hold on;
    title('APF-RRT* + Pruning in Mappa Urbana 3D');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    disp('Disegno della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    plot3(waypoints_raw(:,1), waypoints_raw(:,2), waypoints_raw(:,3), 'Color', [1 0.3 0.3], 'LineWidth', 1.5, 'LineStyle', '--');
    plot3(waypoints_pruned(:,1), waypoints_pruned(:,2), waypoints_pruned(:,3), 'Color', 'g', 'LineWidth', 4);
    plot3(waypoints_pruned(:,1), waypoints_pruned(:,2), waypoints_pruned(:,3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
    
    plot3(q_start.coord(1), q_start.coord(2), q_start.coord(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(q_goal.coord(1), q_goal.coord(2), q_goal.coord(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
    
    view(30,30); axis equal; grid on; hold off;
end