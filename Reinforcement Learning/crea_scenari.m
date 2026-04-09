%% Script Principale: Generazione Database Scenari per RL
clear; clc;

% Impostazioni
num_scenari = 5; %50; % Numero di città/traiettorie da pre-calcolare
scenari = struct(); % Si inizializza la struttura dati vuota

disp(['Avvio generazione di ', num2str(num_scenari), ' scenari...']);

for i = 1:num_scenari
    fprintf('---- Generazione Scenario %d di %d...\n', i, num_scenari);
    
    try

        % 1. Generazione città
        [n_collision, x_max, y_max, z_max] = deal(500, 8000, 8000, 1000);
        [v, q_start, q_goal] = crea_citta(false, false, n_collision, x_max, y_max, z_max);
        map = pack_struct(v, n_collision, x_max, y_max, z_max, q_start, q_goal);
              
        % 2. Path Planning
        [waypoints_raw, waypoints_pruned, traguardo_raggiunto] = run_path_planning(false, false, map);
        
        if traguardo_raggiunto
            % 3. Ottimizzazione minimum snap
            [ground_truth_trajectory] = MinimumSnapCorridors_3D(false, false, waypoints_pruned, map);
            
            % 4. Estrazione timeseries
            [sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory);
            
            % 5. Salvataggio dei dati nella struttura
            scenari(i).map = map;
            scenari(i).sim_pos_des = sim_pos_des;
            scenari(i).sim_vel_des = sim_vel_des;
            scenari(i).sim_yaw_des = sim_yaw_des;
        
        disp('✅ Scenario creato con successo.');
        else
            disp('❌ Scenario non creato, path non raggiunto. Salto al successivo.');
            continue;
        end
        
    catch ME
        % In caso di errore
        % Si salta lo scenario corrente e si passa al successivo
        warning('❌ Errore nello scenario %d: %s. Salto al successivo.', i, ME.message);
        continue;
    end
end

% % Pulizia: rimuove eventuali righe vuote se RRT ha fallito in qualche ciclo
% scenari = scenari(~cellfun(@isempty, {scenari.map})); 

%% Salvataggio su Disco
nome_file = 'training_scenarios.mat';
save(nome_file, 'scenari', '-v7.3'); % -v7.3 è utile se i file superano i 2GB
disp(['🎉 Generazione completata! Database salvato in: ', nome_file]);