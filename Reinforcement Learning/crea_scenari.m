%% Script Principale: Generazione Database Scenari per RL
clear; clc;

% --- FASE 1: GENERAZIONE SCENARI GREZZI ---
% Impostazioni
num_scenari = 10; %50; % Numero di città/traiettorie da pre-calcolare
scenari = struct(); % Si inizializza la struttura dati vuota

num_dyn_obs = 4; % Numero di ostacoli dinamici

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

            % 5. Creazione ostacoli dinamici
            dynamic_obstacles = genera_ostacoli_dinamici(sim_pos_des, num_dyn_obs);
            
            % 6. Salvataggio dei dati nella struttura
            scenari(i).map = map;
            scenari(i).sim_pos_des = sim_pos_des;
            scenari(i).sim_vel_des = sim_vel_des;
            scenari(i).sim_yaw_des = sim_yaw_des;
            scenari(i).dynamic_obstacles = dynamic_obstacles;
        
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

% Rimuovi eventuali scenari vuoti (falliti per "path non raggiunto" o errori)
scenari = scenari(~cellfun(@isempty, {scenari.sim_pos_des}));

% --- FASE 2: UNIFORMAZIONE A POSTERIORI (PADDING) ---
disp('Avvio uniformazione delle timeseries per Fast Restart...');

% 1. Trova la lunghezza massima (MAX_STEPS) in tutto il database
MAX_STEPS = 0;
for i = 1:length(scenari)
    % Usiamo .Length perché sono oggetti timeseries
    lunghezza_attuale = scenari(i).sim_pos_des.Length; 
    if lunghezza_attuale > MAX_STEPS
        MAX_STEPS = lunghezza_attuale;
    end
end
fprintf('La traiettoria più lunga dura %d step. Eseguo il padding...\n', MAX_STEPS);

% 2. Applica il padding a tutti gli scenari
for i = 1:length(scenari)
    lunghezza_attuale = scenari(i).sim_pos_des.Length;
    steps_mancanti = MAX_STEPS - lunghezza_attuale;
    
    if steps_mancanti > 0
        % -- Estrazione dati correnti --
        t_corrente    = scenari(i).sim_pos_des.Time;
        pos_corrente  = scenari(i).sim_pos_des.Data;
        vel_corrente  = scenari(i).sim_vel_des.Data;
        yaw_corrente  = scenari(i).sim_yaw_des.Data;
        
        % -- Padding del Tempo (estrapolazione lineare) --
        dt = t_corrente(2) - t_corrente(1); % Passo di campionamento
        t_mancante = (t_corrente(end) + dt : dt : t_corrente(end) + dt * steps_mancanti)';
        t_nuovo = [t_corrente; t_mancante];
        
        % -- Padding Posizione (Hovering sul traguardo) --
        pos_mancante = repmat(pos_corrente(end, :), steps_mancanti, 1);
        pos_nuova = [pos_corrente; pos_mancante];
        
        % -- Padding Velocità (Fermo a zero) --
        vel_mancante = zeros(steps_mancanti, 3);
        vel_nuova = [vel_corrente; vel_mancante];
        
        % -- Padding Yaw (Mantiene l'ultimo orientamento) --
        yaw_mancante = repmat(yaw_corrente(end, :), steps_mancanti, 1);
        yaw_nuova = [yaw_corrente; yaw_mancante];
        
        % -- Ricostruzione e salvataggio degli oggetti Timeseries --
        scenari(i).sim_pos_des = timeseries(pos_nuova, t_nuovo);
        scenari(i).sim_vel_des = timeseries(vel_nuova, t_nuovo);
        scenari(i).sim_yaw_des = timeseries(yaw_nuova, t_nuovo);
    end
end

disp('✅ Padding completato con successo!');

%% --- FASE 3: SALVATAGGIO SU DISCO ---
nome_file = 'training_scenarios.mat';
save(nome_file, 'scenari', '-v7.3'); % -v7.3 è utile se i file superano i 2GB
disp(['🎉 Generazione completata! Database salvato in: ', nome_file]);

%% Script di verifica dimensioni Timeseries
num_scenari = length(scenari);
fprintf('\nAnalisi di %d scenari...\n', num_scenari);
fprintf('----------------------------------------------------------\n');
fprintf('Scen | Pos.Length | Vel.Length | Yaw.Length | dt medio (s)\n');
fprintf('----------------------------------------------------------\n');

tutte_uguali = true;
% Prendiamo il primo scenario come riferimento
lunghezza_riferimento = scenari(1).sim_pos_des.Length; 
dt_riferimento = mean(diff(scenari(1).sim_pos_des.Time));

for i = 1:num_scenari
    % Estrai le lunghezze usando la proprietà degli oggetti timeseries
    len_pos = scenari(i).sim_pos_des.Length;
    len_vel = scenari(i).sim_vel_des.Length;
    len_yaw = scenari(i).sim_yaw_des.Length;
    
    % Calcola il dt medio dal vettore tempo
    t = scenari(i).sim_pos_des.Time;
    if length(t) > 1
        dt_medio = mean(diff(t));
    else
        dt_medio = 0;
    end
    
    % Stampa i dati formattati in tabella
    fprintf('%4d | %10d | %10d | %10d | %8.4f\n', ...
            i, len_pos, len_vel, len_yaw, dt_medio);
        
    % Controlla se c'è una qualsiasi discordanza interna o col riferimento
    if len_pos ~= lunghezza_riferimento || ...
       len_vel ~= lunghezza_riferimento || ...
       len_yaw ~= lunghezza_riferimento
        tutte_uguali = false;
    end
    
    % Controllo opzionale ma vitale sul tempo
    if abs(dt_medio - dt_riferimento) > 1e-4
        tutte_uguali = false;
    end
end
fprintf('----------------------------------------------------------\n');

% Verdetto finale
if tutte_uguali
    disp('✅ RISULTATO: Perfetto! Tutte le timeseries e i dt sono IDENTICI.');
else
    disp('❌ RISULTATO: Attenzione! Ci sono discordanze nelle dimensioni o nel dt.');
    disp('              (Il Fast Restart non funzionerà finché non saranno allineati)');
end