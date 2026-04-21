function scenari = crea_scenari_grezzi(livello, num_scenari, n_collision, x_max, y_max, z_max, num_dyn_obs, z_threshold)

    if nargin < 8, z_threshold = 1.5; end

    scenari = struct();

%% DATASET CON TRAIETTORIA SEMPLICE O COMPLESSA E OSTACOLI DINAMICI
    if livello == 1 || livello == 2

        count = 0;

        v = zeros(8, 3, 1);
        n_collision = 1;
        frazione_margine = 15;

        disp(['Avvio generazione di ', num2str(num_scenari), ' scenari ']);
        
        while count < num_scenari
            fprintf('\n---- Generazione Scenario %d...\n', count);
            
            x_min_margine = round(x_max/frazione_margine);
            x_max_margine = round(x_max - x_min_margine);
            y_min_margine = round(y_max/frazione_margine);
            y_max_margine = round(y_max - y_min_margine);
            z_min_margine = round(z_max/frazione_margine);
            z_max_margine = round(z_max - z_min_margine);

            
            q_start = [randi([x_min_margine, x_max_margine]), randi([y_min_margine, y_max_margine]), randi([z_min_margine, z_max_margine])];
            q_goal = q_start;
            while norm(q_goal - q_start) < min(y_max, x_max) / 1.5
                q_goal = [randi([x_min_margine, x_max_margine]), randi([y_min_margine, y_max_margine]), randi([z_min_margine, z_max_margine])];
            end
            
            map = pack_struct(v, n_collision, x_max, y_max, z_max, q_start, q_goal);
            
            [waypoints_raw, waypoints_pruned, traguardo_raggiunto] = run_path_planning(false, false, map);
                    
            if traguardo_raggiunto
                % 3. Ottimizzazione minimum snap
                [ground_truth_trajectory] = MinimumSnapCorridors_3D(false, true, waypoints_pruned, map);
                
                % 4. Estrazione timeseries
                [sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory);
                
                % 4. Creazione ostacoli dinamici
                dynamic_obstacles = genera_ostacoli_dinamici(sim_pos_des, num_dyn_obs);
            else
                disp('ERRORE');
                return
            end
            
            % 5. Salvataggio dei dati nella struttura
            count = count + 1;
            
            scenari(count).map = map;
            scenari(count).sim_pos_des = sim_pos_des;
            scenari(count).sim_vel_des = sim_vel_des;
            scenari(count).sim_yaw_des = sim_yaw_des;
            scenari(count).dynamic_obstacles = dynamic_obstacles;
        
            disp('✅ Scenario creato con successo.');
        end

%% DATASET CON EDIFICI, TRAIETTORIE COMPLESSE E OSTACOLI DINAMICI
% ... TODO ...

%% DATASET CON EDIFICI, TRAIETTORIE COMPLESSE FINO AD ATTERRAGGIO, E OSTACOLI DINAMICI
% ... TODO ... 

%% DATASET ORIGINALE
    elseif livello == 4 

        disp(['Avvio generazione di ', num2str(num_scenari), ' scenari validi...']);

        valid_count = 0; % Contatore degli scenari validi generati
        attempts = 0;    % Contatore dei tentativi totali
        
        while valid_count < num_scenari
            attempts = attempts + 1;
            fprintf('\n---- Generazione Scenario %d di %d (Tentativo totale %d)...\n', valid_count + 1, num_scenari, attempts);
            
            try
                % 1. Generazione città
                [v, q_start, q_goal] = crea_citta(false, false, n_collision, x_max, y_max, z_max);
                map = pack_struct(v, n_collision, x_max, y_max, z_max, q_start, q_goal);
                      
                % 2. Path Planning
                [waypoints_raw, waypoints_pruned, traguardo_raggiunto] = run_path_planning(false, false, map);
                
                if traguardo_raggiunto
                    % 3. Ottimizzazione minimum snap
                    [ground_truth_trajectory] = MinimumSnapCorridors_3D(false, true, waypoints_pruned, map);
                    
                    % 4. Estrazione timeseries
                    [sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory);
                    
                    % --- FILTRO TRAIETTORIE BANALI ---
                    % Estraiamo le quote Z dalla traiettoria desiderata
                    z_data = sim_pos_des.Data(:, 3);
                    
                    % Se l'altezza massima raggiunta è troppo vicina a z=1, scarta lo scenario
                    if max(z_data) < z_threshold
                        disp('⚠️ Traiettoria banale rilevata (Z pressoché costante). Scarto e rigenero.');
                        % Chiude l'ultima figura aperta (quella generata da MinimumSnapCorridors_3D)
                        if ~isempty(get(0, 'CurrentFigure'))
                            close(gcf); 
                        end
                        continue; % Salta il resto e inizia un nuovo tentativo
                    end
                    
                    % 5. Creazione ostacoli dinamici
                    dynamic_obstacles = genera_ostacoli_dinamici(sim_pos_des, num_dyn_obs);
                    
                    % 6. Salvataggio dei dati nella struttura
                    valid_count = valid_count + 1; % Incrementa solo se lo scenario supera tutti i controlli
                    
                    scenari(valid_count).map = map;
                    scenari(valid_count).sim_pos_des = sim_pos_des;
                    scenari(valid_count).sim_vel_des = sim_vel_des;
                    scenari(valid_count).sim_yaw_des = sim_yaw_des;
                    scenari(valid_count).dynamic_obstacles = dynamic_obstacles;
                
                    disp('✅ Scenario creato con successo.');
                else
                    disp('❌ Scenario non creato, path non raggiunto. Salto al successivo.');
                    continue;
                end
                
            catch ME
                % In caso di errore
                warning('❌ Errore durante il tentativo %d: %s. Salto al successivo.', attempts, ME.message);
                continue;
            end
        end

    end
end