function [v, q_start, q_goal] = crea_citta(enable_save, enable_plot, n_collision, x_max, y_max, z_max)
    % GENERATE_URBAN_SCENARIO Genera una mappa 3D e trova start/goal sicuri.
    %
    % Output:
    %   v - Matrice dei vertici (8 x 3 x n_collision)
    %   q_start, q_goal - Coordinate dei punti generati

    % 1. Impostazione dei default
    if nargin < 1, enable_save = true; end
    if nargin < 2, enable_plot = true; end
    if nargin < 3, n_collision = 500; end
    if nargin < 4, x_max = 8000; end
    if nargin < 5, y_max = 8000; end
    if nargin < 6, z_max = 1000; end
    
    disp('Inizializzazione Generatore Urbano Randomico...');
    tic
    rng('shuffle'); 
    
    % 2. Parametrizzazione Dinamica in base alle dimensioni della mappa
    min_dim = min(x_max, y_max);
    
    city_params.edge_offset = max(50, min_dim * 0.05); % 5% della dimensione minima
    city_params.min_pos = city_params.edge_offset;
    city_params.min_len = max(20, min_dim * 0.0125);   % Dimensione minima edificio
    city_params.max_len = max(50, min_dim * 0.0375);   % Dimensione massima edificio
    city_params.overlap_margin = max(10, min_dim * 0.005); 
    
    % 3. Controllo Sicurezza Densità
    avg_area = ((city_params.min_len + city_params.max_len) / 2)^2;
    usable_area = (x_max - 2*city_params.edge_offset) * (y_max - 2*city_params.edge_offset);
    max_coverage = 0.45; % Limite massimo 45% dello spazio calpestabile
    max_buildings = max(1, floor((usable_area * max_coverage) / avg_area));
    
    if n_collision > max_buildings
        warning('Densità troppo elevata! Riduzione edifici da %d a %d per evitare congestione e loop.', n_collision, max_buildings);
        n_collision = max_buildings;
    end
    
    map_params.x_max = x_max; 
    map_params.y_max = y_max; 
    map_params.z_max = z_max;
    map_params.n_collision = n_collision;

    % 4. Generazione procedurale della città
    disp('Costruzione degli edifici in corso...');
    [v, ~, ~] = generate_urban_map(map_params, city_params);
    
    % Aggiorniamo n_collision in caso il timeout della generazione lo abbia ridotto
    n_collision = size(v, 3);
    map_params.n_collision = n_collision;

    % 5. Parametri per Start e Goal
    sg_params.margin = max(15.0, min_dim * 0.01);      
    sg_params.min_dist = min(x_max/2, y_max/2); % Corretto typo originale 'y_min'    
    sg_params.z_start = 1.0;      
    sg_params.z_goal = 1.0;       
    sg_params.edge_offset = city_params.edge_offset; % Margine dai bordi per i punti

    % 6. Ricerca di Start e Goal sicuri
    disp('Ricerca di Start e Goal sicuri...');
    [q_start, q_goal] = find_start_goal(v, map_params, sg_params);
    
    disp(['🟢 Start valido a: [', num2str(q_start, '%.1f '), ']']);
    disp(['🟣 Goal valido a : [', num2str(q_goal, '%.1f '), ']']);
    disp(['📏 Distanza retta : ', num2str(dist_3d(q_start, q_goal), '%.1f'), ' metri']);
    disp(['⏱ Tempo totale di generazione: ', num2str(toc), ' secondi']);

    % 7. Grafica (Condizionale)
    if enable_plot
        plot_generated_city(v, n_collision, q_start, q_goal);
    end

    % 8. Salvataggio (Condizionale)
    if enable_save
        save('mappa_urbana.mat', 'v', 'n_collision', 'x_max', 'y_max', 'z_max', 'q_start', 'q_goal');
        disp('💾 Mappa e punti strategici salvati in "mappa_urbana.mat".');
    end
end

% --- FUNZIONI SECONDARIE ---

function [v, creat_center, lengthxyz] = generate_urban_map(map_params, city_params)
    n_col_req = map_params.n_collision;
    z_max = map_params.z_max;
    
    creat_center = zeros(n_col_req, 3);
    lengthxyz = zeros(n_col_req, 3);
    v = zeros(8, 3, n_col_req);
    
    min_pos = city_params.min_pos; 
    max_pos_x = map_params.x_max - city_params.edge_offset; 
    max_pos_y = map_params.y_max - city_params.edge_offset; 
    min_len = city_params.min_len; 
    max_len = city_params.max_len; 
    margine = city_params.overlap_margin; 
    
    i = 1;
    attempts = 0;
    max_attempts = n_col_req * 100; % Timeout di sicurezza
    
    while i <= n_col_req && attempts < max_attempts
        attempts = attempts + 1;
        
        cand_center = [min_pos + (max_pos_x - min_pos) * rand(), ...
                       min_pos + (max_pos_y - min_pos) * rand(), 0]; 
        cand_len = [min_len + (max_len - min_len) * rand(1, 2), 0];
        
        % Altezze scalate in base a z_max invece di valori costanti
        prob = rand();
        if prob < 0.60
            cand_len(3) = z_max * 0.1 + (z_max * 0.15) * rand();       
        elseif prob < 0.90
            cand_len(3) = z_max * 0.25 + (z_max * 0.3) * rand();      
        else
            cand_len(3) = z_max * 0.55 + (z_max * 0.4) * rand();      
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
            
            % Generazione Vertici immediata
            l_x = cand_len(1); l_y = cand_len(2); l_z = cand_len(3);
            cx = cand_center(1); cy = cand_center(2);
            
            v(:,:,i) = [cx+l_x, cy+l_y, l_z; 
                        cx,     cy+l_y, l_z; 
                        cx,     cy,     l_z; 
                        cx+l_x, cy,     l_z; 
                        cx+l_x, cy+l_y, 0; 
                        cx,     cy+l_y, 0; 
                        cx,     cy,     0; 
                        cx+l_x, cy,     0];
            i = i + 1;
        end
    end
    
    n_effettivi = i - 1;
    if n_effettivi < n_col_req
        disp(['⚠️ Raggiunto limite tentativi. Edifici generati: ', num2str(n_effettivi), '/', num2str(n_col_req)]);
        % Tronca le matrici se non è riuscito a posizionare tutti i palazzi
        v = v(:,:,1:n_effettivi);
        creat_center = creat_center(1:n_effettivi, :);
        lengthxyz = lengthxyz(1:n_effettivi, :);
    end
end

function [q_start, q_goal] = find_start_goal(v, map_params, sg_params)
    offset = sg_params.edge_offset;
    x_range = map_params.x_max - 2 * offset;
    y_range = map_params.y_max - 2 * offset;

    % --- Trova uno START sicuro ---
    valido_start = false;
    while ~valido_start
        cand_x = offset + rand() * x_range;
        cand_y = offset + rand() * y_range;
        cand_start = [cand_x, cand_y, sg_params.z_start];
        
        if ~is_colliding(cand_start, v, sg_params.margin)
            q_start = cand_start;
            valido_start = true;
        end
    end
    
    % --- Trova un GOAL sicuro e lontano ---
    valido_goal = false;
    while ~valido_goal
        cand_x = offset + rand() * x_range;
        cand_y = offset + rand() * y_range;
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
    title('Mappa Urbana 3D Randomica');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    disp('Rendering 3D in corso...');
    
    f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    for k = 1:n_collision
        patch('Faces', f, 'Vertices', v(:,:,k), 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    plot3(q_start(1), q_start(2), q_start(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(q_goal(1), q_goal(2), q_goal(3), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
    view(30,30); axis equal; grid on; hold off;
end