function graphic_func(log_posizione)
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
    hold on; grid on; view(3);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Traiettoria 3D del Drone');
    
    % Disegna Start e End
    plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End/Crash');
    
    axis([min(X)-5, max(X)+5, min(Y)-5, max(Y)+5, min(Z)-2, max(Z)+5]);
    
    % Animazione
    curve = animatedline('Color', 'b', 'LineWidth', 2, 'DisplayName', 'Traiettoria');
    legend('Location', 'best');
    
    step_salto = max(1, floor(length(X) / 200)); 
    for i = 1:step_salto:length(X)
        addpoints(curve, X(i), Y(i), Z(i));
        drawnow; 
        pause(0.01); 
    end
    addpoints(curve, X(end), Y(end), Z(end));
    drawnow;
    disp('Replay concluso!');
end