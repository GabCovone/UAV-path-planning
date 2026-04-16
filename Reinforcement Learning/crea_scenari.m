%% Script Principale: Generazione Database Scenari per RL
clear; clc;

% --- FASE 1: GENERAZIONE SCENARI GREZZI ---
% Impostazioni
num_scenari = 50; % Numero di città/traiettorie da pre-calcolare
scenari = struct(); % Si inizializza la struttura dati vuota

n_collision = 40; % Numero edifici, di base 500
x_max = 2000;
y_max = 2000;
z_max = 1000;
num_dyn_obs = 10; % Numero di ostacoli dinamici

% Parametro per il filtro delle traiettorie banali
z_threshold = 1.5; % Quota minima che il drone deve superare per non essere considerato "banale"

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

% NOTA: La pulizia delle celle vuote è stata rimossa perché il ciclo 'while' 
% popola la struttura 'scenari' in modo strettamente sequenziale solo in caso di successo.

% --- FASE 2: UNIFORMAZIONE A POSTERIORI (PADDING) ---
disp('\nAvvio uniformazione delle timeseries per Fast Restart...');

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
num_scenari_generati = length(scenari);
fprintf('\nAnalisi di %d scenari...\n', num_scenari_generati);
fprintf('----------------------------------------------------------\n');
fprintf('Scen | Pos.Length | Vel.Length | Yaw.Length | dt medio (s)\n');
fprintf('----------------------------------------------------------\n');

tutte_uguali = true;
% Prendiamo il primo scenario come riferimento
lunghezza_riferimento = scenari(1).sim_pos_des.Length; 
dt_riferimento = mean(diff(scenari(1).sim_pos_des.Time));

for i = 1:num_scenari_generati
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