% --- VISUALIZZATORE VOXEL LOCALI ---

% 1. Trova un istante in cui il drone vede almeno un ostacolo
% (Cerchiamo un frame in cui la somma dei primi 1000 valori è maggiore di 0)
ostacoli_visti = sum(Transitions(:, 1:1000), 2);
frame_interessante = find(ostacoli_visti > 10, 1); % Trova un frame con almeno 10 voxel di muro

if isempty(frame_interessante)
    disp('Nessun ostacolo visto in questa simulazione! Il drone ha volato nel vuoto.');
else
    disp(['Trovato un frame interessante all''istante ', num2str(frame_interessante)]);
    
    % 2. Estrai il vettore piatto e rigonfialo a cubo 3D (10x10x10)
    voxel_flat = Transitions(frame_interessante, 1:1000);
    cubo = reshape(voxel_flat, [10, 10, 10]);
    
    % 3. Trova le coordinate tridimensionali dei voxel "pieni" (pari a 1)
    [X, Y, Z] = ind2sub(size(cubo), find(cubo == 1));
    
    % 4. Creazione del Grafico 3D
    figure('Name', 'Visione Voxel del Drone', 'Color', 'w');
    hold on;
    
    % Disegna i voxel del palazzo come cubetti rossi
    if ~isempty(X)
        scatter3(X, Y, Z, 800, 's', 'filled', 'MarkerFaceColor', [0.8 0.2 0.2], 'MarkerEdgeColor', 'k');
    end
    
    % Disegna il drone esattamente al centro del cubo (posizione 5.5, 5.5, 5.5)
    scatter3(5.5, 5.5, 5.5, 200, 'o', 'filled', 'MarkerFaceColor', 'b');
    text(5.5, 5.5, 6.5, '  Drone', 'Color', 'b', 'FontWeight', 'bold');
    
    % Formattazione grafica
    grid on; axis equal;
    xlim([1 10]); ylim([1 10]); zlim([1 10]);
    title(['Visione Locale (10x10x10) - Frame: ', num2str(frame_interessante)]);
    xlabel('X Voxel'); ylabel('Y Voxel'); zlabel('Z Voxel');
    view(30, 30);
    hold off;
end