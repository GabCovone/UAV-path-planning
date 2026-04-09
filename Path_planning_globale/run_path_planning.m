function [waypoints_raw, waypoints_pruned, traguardo_raggiunto] = run_path_planning(enable_save, enable_plot, map)
    % RUN_URBAN_SIMULATION Esegue l'APF-RRT* su una mappa esistente
    %
    % Parametri di ingresso (Opzionali):
    %   enable_save - Booleano. Se true, salva in 'waypoints_estratti.mat' (Default: true)
    %   enable_plot - Booleano. Se true, mostra l'albero e il pruning (Default: true)
    %   map_file    - Stringa. Il nome del file .mat da cui caricare la mappa (Default: 'mappa_urbana.mat')
    %
    % Output:
    %   waypoints_raw    - I nodi grezzi estratti dall'albero RRT*
    %   waypoints_pruned - I punti pruned finali (line-of-sight)

    % Impostazione dei default se i parametri non vengono passati
    if nargin < 1, enable_save = true; end
    if nargin < 2, enable_plot = true; end
    if nargin < 3, map = 'mappa_urbana.mat'; end
    
    disp('Inizializzazione simulatore urbano...');
    tic
    
    % 1. Inizializzazione e caricamento mappa
    [v, n_collision, q_start, q_goal, frame_range, v_center, v_r] = init_environment(map);
    
    % Parametri APF-RRT*
    params.numNodes = 30000;
    params.r_create = 800;
    params.pho = 50;           % Lunghezza del passo
    params.k_att = 1.5;        % Attrazione
    params.k_rep = 1.5;        % Repulsione
    params.r_rewire = 150;     % Raggio di rewiring RRT*
    params.goal_tol = 55;      % Tolleranza arrivo
    params.margin = 10.0;      % Margine ostacoli
    params.step_size = 5.0;    % Risoluzione collision checking
    
    % 2. Ricerca del percorso (APF-RRT*)
    disp('Ricerca del percorso ottimale (RRT*) in corso...');
    [nodes, node_count, traguardo_raggiunto] = run_apf_rrt_star(q_start, q_goal, v, n_collision, v_center, v_r, frame_range, params);
    
    if traguardo_raggiunto
        % 3. Estrazione del percorso grezzo
        [waypoints_raw, nodes, node_count] = extract_raw_path(nodes, node_count, q_goal);
        
        % 4. Ottimizzazione del percorso (Pruning)
        disp('Esecuzione Pruning (Line-of-Sight)...');
        waypoints_pruned = prune_path_los(waypoints_raw, v, n_collision, params);
        disp(['Tempo di calcolo totale: ', num2str(toc), ' secondi']);
        
        % 5. Grafica (Condizionale)
        if enable_plot
            plot_urban_path(waypoints_raw, waypoints_pruned, q_start, q_goal, v, n_collision);
        else
            disp('Grafica disabilitata tramite parametro.');
        end
        
        % 6. Salvataggio (Condizionale)
        if enable_save
            save('waypoints_estratti.mat', 'waypoints_pruned');
            disp('Waypoints salvati con successo in "waypoints_estratti.mat".');
        else
            disp('Salvataggio dati disabilitato tramite parametro.');
        end
    else
        waypoints_raw = NaN;
        waypoints_pruned = NaN;
        disp('Traguardo non raggiunto');
    end
end