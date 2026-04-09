function [v, q_start, q_goal] = crea_citta(enable_save, enable_plot, n_collision, x_max, y_max, z_max)
    % GENERATE_URBAN_SCENARIO Genera una mappa 3D e trova start/goal sicuri.
    %
    % Parametri di ingresso (Opzionali):
    %   enable_save - Booleano (true/false). Se true, salva i dati in 'mappa_urbana.mat' (Default: true)
    %   enable_plot - Booleano (true/false). Se true, renderizza la mappa 3D (Default: true)
    %
    % Output:
    %   v, n_collision - Matrice dei vertici della città e numero di ostacoli
    %   q_start, q_goal - Coordinate dei punti generati

    % Impostazione dei default se i parametri non vengono passati
    if nargin < 1, enable_save = true; end
    if nargin < 2, enable_plot = true; end
    if nargin < 3, n_collision = 500; end
    if nargin < 4, x_max = 8000; end
    if nargin < 5, y_max = 8000; end
    if nargin < 6, z_max = 1000; end

    disp('Inizializzazione Generatore Urbano Randomico...');
    tic

    rng('shuffle'); 

    % 1. Parametri della mappa e della città
    map_params.x_max = x_max; 
    map_params.y_max = y_max; 
    map_params.z_max = z_max;
    map_params.n_collision = n_collision;

    city_params.min_pos = 100;
    city_params.edge_offset = 400;   
    city_params.min_len = 100;
    city_params.max_len = 300;
    city_params.overlap_margin = 30; 

    % 2. Generazione procedurale della città
    disp('Costruzione dei grattacieli in corso...');
    [v, ~, ~] = generate_urban_map(map_params, city_params);

    % 3. Parametri per Start e Goal
    sg_params.margin = 15.0;      
    sg_params.min_dist = 4000;    
    sg_params.z_start = 1.0;      
    sg_params.z_goal = 1.0;       

    % 4. Ricerca di Start e Goal sicuri
    disp('Ricerca di Start e Goal sicuri...');
    [q_start, q_goal] = find_start_goal(v, map_params, sg_params);

    disp(['🟢 Start valido a: [', num2str(q_start, '%.1f '), ']']);
    disp(['🟣 Goal valido a : [', num2str(q_goal, '%.1f '), ']']);
    disp(['📏 Distanza retta : ', num2str(dist_3d(q_start, q_goal), '%.1f'), ' metri']);
    disp(['Tempo totale di generazione: ', num2str(toc), ' secondi']);

    % 5. Grafica (Condizionale)
    if enable_plot
        plot_generated_city(v, map_params.n_collision, q_start, q_goal);
    else
        disp('Grafica disabilitata tramite parametro.');
    end

    % 6. Salvataggio (Condizionale)
    if enable_save
        x_max = map_params.x_max; 
        y_max = map_params.y_max; 
        z_max = map_params.z_max;
        n_collision = map_params.n_collision;
        
        save('mappa_urbana.mat', 'v', 'n_collision', 'x_max', 'y_max', 'z_max', 'q_start', 'q_goal');
        disp('💾 Mappa e punti strategici salvati in "mappa_urbana.mat".');
    else
        disp('Salvataggio dati disabilitato tramite parametro.');
    end
end

% Funzioni secondarie

function [v, creat_center, lengthxyz] = generate_urban_map(map_params, city_params)
    n_collision = map_params.n_collision;
    max_xy = map_params.x_max; % Assumiamo mappa quadrata per semplicità
    
    creat_center = zeros(n_collision, 3);
    lengthxyz = zeros(n_collision, 3);
    v = zeros(8, 3, n_collision);
    
    min_pos = city_params.min_pos; 
    max_pos = max_xy - city_params.edge_offset; 
    min_len = city_params.min_len; 
    max_len = city_params.max_len; 
    margine = city_params.overlap_margin; 
    
    i = 1;
    while i <= n_collision
        cand_center = [min_pos + (max_pos - min_pos) * rand(1, 2), 0]; 
        cand_len = [min_len + (max_len - min_len) * rand(1, 2), 0];
        
        prob = rand();
        if prob < 0.60
            cand_len(3) = 50 + 100 * rand();       
        elseif prob < 0.90
            cand_len(3) = 150 + 250 * rand();      
        else
            cand_len(3) = 400 + 400 * rand();      
        end
        
        cand_min = cand_center(1:2);
        cand_max = cand_center(1:2) + cand_len(1:2);
        
        if i > 1
            esist_min = creat_center(1:i-1, 1:2);
            esist_max = creat_center(1:i-1, 1:2) + lengthxyz(1:i-1, 1:2);
            
            overlap_x = (cand_min(1) < esist_max(:,1) + margine) & (cand_max(1) + margine > esist_min(:,1));
            overlap_y = (cand_min(2) < esist_max(:,2) + margine) & (cand_max(2) + margine > esist_min(:,2));
            sovrapposto = any(overlap_x & overlap_y);
        else
            sovrapposto = false;
        end
        
        if ~sovrapposto
            creat_center(i, :) = cand_center;
            lengthxyz(i, :) = cand_len;
            i = i + 1;
        end
    end
    
    % Generazione Vertici
    for k = 1:n_collision
        point1 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2)+lengthxyz(k,2), lengthxyz(k,3)];
        point2 = [creat_center(k,1),                creat_center(k,2)+lengthxyz(k,2), lengthxyz(k,3)];
        point3 = [creat_center(k,1),                creat_center(k,2),                lengthxyz(k,3)];
        point4 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2),                lengthxyz(k,3)];
        point5 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2)+lengthxyz(k,2), 0];
        point6 = [creat_center(k,1),                creat_center(k,2)+lengthxyz(k,2), 0];
        point7 = [creat_center(k,1),                creat_center(k,2),                0];
        point8 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2),                0];
        v(:,:,k) = [point1; point2; point3; point4; point5; point6; point7; point8];
    end
end

% necessita di dist_3d e is_colliding nel path
function [q_start, q_goal] = find_start_goal(v, map_params, sg_params)
    % --- Trova uno START sicuro ---
    valido_start = false;
    while ~valido_start
        cand_x = 200 + rand() * (map_params.x_max - 400);
        cand_y = 200 + rand() * (map_params.y_max - 400);
        cand_start = [cand_x, cand_y, sg_params.z_start];
        
        if ~is_colliding(cand_start, v, sg_params.margin)
            q_start = cand_start;
            valido_start = true;
        end
    end
    
    % --- Trova un GOAL sicuro e lontano ---
    valido_goal = false;
    while ~valido_goal
        cand_x = 200 + rand() * (map_params.x_max - 400);
        cand_y = 200 + rand() * (map_params.y_max - 400);
        cand_goal = [cand_x, cand_y, sg_params.z_goal];
        
        dist_sg = dist_3d(q_start, cand_goal);
        
        if ~is_colliding(cand_goal, v, sg_params.margin) && dist_sg > sg_params.min_dist
            q_goal = cand_goal;
            valido_goal = true;
        end
    end
end

function plot_generated_city(v, n_collision, q_start, q_goal)
    figure(1); hold on;
    title('Mappa Urbana 3D Randomica con Start/Goal Sicuri');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    disp('Rendering 3D in corso...');
    
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    % Disegna i marker di Start (Verde) e Goal (Magenta)
    plot3(q_start(1), q_start(2), q_start(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(q_goal(1), q_goal(2), q_goal(3), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
    view(30,30); axis equal; grid on; hold off;
end