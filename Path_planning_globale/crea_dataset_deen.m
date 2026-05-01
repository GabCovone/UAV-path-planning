%% Script Completo e Ottimizzato: Generazione Dataset per DEEN
clear; clc; close all;

%% =======================================================================
% 1. IMPOSTAZIONI E PARAMETRI GENERALI
% ========================================================================
num_scenari_target = 5;    % Quanti scenari VALIDI vogliamo generare (aumentare per il training finale)
livello = 3;                  % Livello mappa (es. 3 o 4 per città con ostacoli)
z_threshold = 4.0;            % Quota minima (Z) per non scartare traiettorie banali
dim_drone = 0.3;              % Raggio del drone (metri) per calcolo collisioni
max_range_lidar = 20.0;       % Range massimo del Raytracing
num_rays = 256;               % Numero di raggi della Sfera di Fibonacci
file_h5_output = 'dataset_deen_ottimizzato.h5';
indici_split = zeros(1, 2);

% Parametri Mappa 
n_collision = 40; 
x_max = 500; y_max = 500; z_max = 500;
num_dyn_obs = 3; 

dynamic_obs.numero = num_dyn_obs;
dynamic_obs.statici = "no";
dynamic_obs.raggi = [1.0 2.0];

disp(['🚀 Avvio generazione Dataset DEEN: Obiettivo ', num2str(num_scenari_target), ' scenari.']);

%% =======================================================================
% 2. PREALLOCAZIONE MEMORIA E BUFFER (OTTIMIZZAZIONE)
% ========================================================================
% Preallochiamo un grosso blocco di memoria in formato 'single'.
% Questo evita la frammentazione della RAM e ottimizza il salvataggio per PyTorch.
max_transizioni_stimate = num_scenari_target * 1000; % Stima: 1000 step per scenario in media
DATASET_TOTALE = zeros(max_transizioni_stimate, 536, 'single');
current_row = 1; % Indice della prima riga libera nel dataset globale

valid_count = 0;
attempts = 0;

%% =======================================================================
% 3. LOOP PRINCIPALE DI GENERAZIONE
% ========================================================================
while valid_count < num_scenari_target
    attempts = attempts + 1;
    fprintf('\n----------------------------------------------------\n');
    fprintf('🎬 Generazione Scenario %d di %d (Tentativo %d)\n', valid_count + 1, num_scenari_target, attempts);
    
    % --- LOGICA NEAR-MISS (80% Sicuro, 20% Rischioso) ---
    if rand() < 0.80
        margine_sicurezza = 1.5; % Volo centrale e largo
        tipo_volo = 'OTTIMALE';
    else
        margine_sicurezza = dim_drone + 0.1; % Volo radente ai muri
        tipo_volo = 'NEAR-MISS (Rischioso)';
    end
    fprintf('   Tipo traiettoria: %s (Margine: %.2f m)\n', tipo_volo, margine_sicurezza);

    try
        % 1. Generazione città e ostacoli statici
        [v, q_start, q_goal] = crea_citta(false, false, n_collision, x_max, y_max, z_max);
        map = pack_struct(v, n_collision, x_max, y_max, z_max, q_start, q_goal);
              
        % 2. Path Planning (con Margine Dinamico)
        % Chiamata ottimizzata: salvataggio (false), plot (false)
        [waypoints_raw, waypoints_pruned, traguardo_raggiunto] = run_path_planning(false, false, map, margine_sicurezza);
        if ~traguardo_raggiunto
            disp('   ❌ Path non trovato. Salto al successivo.');
            continue;
        end
        
        % 3. Ottimizzazione minimum snap (Traiettoria Desiderata)
        % Chiamata ottimizzata: salvataggio (false), plot (false)
        [ground_truth_trajectory] = MinimumSnapCorridors_3D(false, false, waypoints_pruned, map, margine_sicurezza);
        [sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory);
        
        % Filtro traiettorie banali rasoterra
        if max(sim_pos_des.Data(:, 3)) < z_threshold
            disp('   ⚠️ Traiettoria troppo bassa. Scarto e rigenero.');
            continue;
        end
        
        % ================================================================
        % 4. INIZIALIZZAZIONE MODELLO E CONDIZIONI INIZIALI
        % ================================================================
        fprintf('   ⚙️ Inizializzazione Data Dictionary e Workspace...\n');
        
        % 4.1 Chiavi logiche
        assignin('base', 'plantModelFi', 1);            
        assignin('base', 'useHeading', 1);              
        assignin('base', 'initialGainsMultiplier', 15); 
        
        % 4.2 Apertura SLDD e impostazione spawn point (NED)
        start_point = ground_truth_trajectory(1, :);
        dict = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');
        sect = getSection(dict, 'Design Data');
        entry = getEntry(sect, 'initialConditions');
        initStruct = getValue(entry);
        
        % Inversione Z per asse NED
        initStruct.posNED = cast([start_point(1), start_point(2), -start_point(3)], class(initStruct.posNED));
        setValue(entry, initStruct);
        saveChanges(dict);
        
        % 4.3 Estrazione parametri nel Base Workspace (per Simulink)
        exportToFile(sect, 'temp_parametri_drone.mat');
        parametri = load('temp_parametri_drone.mat');
        campi_p = fieldnames(parametri);
        for i = 1:length(campi_p)
            assignin('base', campi_p{i}, parametri.(campi_p{i}));
        end
        
        % 4.4 Estrazione Bus (se esistono)
        try
            sect_interfaces = getSection(dict, 'Interfaces');
            exportToFile(sect_interfaces, 'temp_buses_drone.mat');
            buses = load('temp_buses_drone.mat');
            campi_b = fieldnames(buses);
            for i = 1:length(campi_b)
                assignin('base', campi_b{i}, buses.(campi_b{i}));
            end
        catch
            % Nessun bus trovato nella sezione interfaces, ignoriamo
        end
        close(dict);
        % Pulizia file temporanei
        if isfile('temp_parametri_drone.mat'), delete('temp_parametri_drone.mat'); end
        if isfile('temp_buses_drone.mat'), delete('temp_buses_drone.mat'); end
        
        % 4.5 Passaggio Riferimenti Ottimali
        assignin('base', 'sim_pos_des', sim_pos_des);
        assignin('base', 'sim_vel_des', sim_vel_des);
        assignin('base', 'sim_yaw_des', sim_yaw_des);
        
        % 4.6 Calcolo Dinamico del tempo di simulazione
        tempo_simulazione = sim_pos_des.Time(end);
        fprintf('   🚀 Avvio simulazione (Durata: %.1f sec)...\n', tempo_simulazione);
        
        % SIMULAZIONE IN BACKGROUND (con StopTime dinamico)
        simOut = sim('MultirotorModel', ...
                     'StopTime', num2str(tempo_simulazione), ...
                     'ReturnWorkspaceOutputs', 'on');
        
        estrai_segnale = @(sim_out, nome) helper_estrai(sim_out, nome);
        
        try
            % Estrazione diretta dalle timeseries native di Simulink
            % Usiamo squeeze() per appiattire eventuali matrici 3D [3x1xN] in 2D
            pos_act    = squeeze(simOut.pos_reale.Data);
            vel_act    = squeeze(simOut.vel_reale.Data);
            omega_act  = squeeze(simOut.omega_reale.Data);
            quat_act   = squeeze(simOut.quat_reale.Data);
            act_motori = squeeze(simOut.azioni_motori.Data);
            time_act   = simOut.tout; % Tempo globale della simulazione
            
            % Simulink a volte inverte righe e colonne. 
            % Ci assicuriamo che siano SEMPRE nel formato PyTorch: [Step Temporali x Features]
            if size(pos_act, 2) > size(pos_act, 1), pos_act = pos_act'; end
            if size(vel_act, 2) > size(vel_act, 1), vel_act = vel_act'; end
            if size(omega_act, 2) > size(omega_act, 1), omega_act = omega_act'; end
            if size(quat_act, 2) > size(quat_act, 1), quat_act = quat_act'; end
            if size(act_motori, 2) > size(act_motori, 1), act_motori = act_motori'; end
            
        catch ME
            warning('❌ Impossibile leggere i dati da simOut: %s', ME.message);
            continue;
        end
        
        % Generazione Ostacoli Dinamici basati sul tempo reale
        dynamic_obstacles = genera_ostacoli_dinamici_deen(pos_act, time_act, num_dyn_obs, dynamic_obs.raggi, dynamic_obs.statici);
        
        % ================================================================
        % 5. ESTRAZIONE TRANSIZIONI (IL CUORE DEL DATASET)
        % ================================================================
        fprintf('   🔄 Calcolo Raytracing e validazione collisioni...\n');
        
        num_steps = length(time_act);
        % Buffer temporaneo per questo episodio (velocizza il loop interno)
        buffer_episodio = zeros(num_steps-1, 536, 'single'); 
        episodio_valido = true;
        min_clearance_volo = inf; % Traccia quanto ci siamo avvicinati
        
        for t = 1:(num_steps - 1)
            % --- STATO ATTUALE (t) ---
            lidar_t = estrai_lidar(pos_act(t, :), map.v, dynamic_obstacles, time_act(t), num_rays);
            kin_t = [vel_act(t, :), omega_act(t, :), quat_act(t, :)]; % 10 elementi
            
            % --- AZIONE (t) ---
            a_t = act_motori(t, :); % 4 elementi
            
            % --- STATO SUCCESSIVO (t+1) ---
            lidar_t1 = estrai_lidar(pos_act(t+1, :), map.v, dynamic_obstacles, time_act(t+1), num_rays);
            kin_t1 = [vel_act(t+1, :), omega_act(t+1, :), quat_act(t+1, :)]; % 10 elementi
            
            % --- CONTROLLO COLLISIONE SUI DATI REALI ---
            distanze_metri = lidar_t1 * max_range_lidar; 
            distanza_minima_step = min(distanze_metri);
            
            % Aggiornamento tracciamento distanza minima
            if distanza_minima_step < min_clearance_volo
                min_clearance_volo = distanza_minima_step;
            end
            
            if any(distanze_metri <= dim_drone)
                fprintf('   💥 SCHIANTO IN SIMULINK! Ostacolo a %.2f m. Scarto l''episodio.\n', min_clearance_volo);
                episodio_valido = false;
                break;
            end
            
            % Composizione della riga da 536 valori (cast a single)
            buffer_episodio(t, :) = single([lidar_t, kin_t, a_t, lidar_t1, kin_t1]);
        end
        
        % ================================================================
        % 6. AGGIUNTA AL DATASET GLOBALE E GESTIONE MEMORIA
        % ================================================================
        if episodio_valido
            righe_valide = t - 1; 
            
            % Espansione dinamica se DATASET_TOTALE è pieno
            if current_row + righe_valide > size(DATASET_TOTALE, 1)
                fprintf('   ⚠️ Espansione memoria dataset...\n');
                chunk_size = num_scenari_target * 500; % Alloca altro spazio
                DATASET_TOTALE = [DATASET_TOTALE; zeros(chunk_size, 536, 'single')]; %#ok<AGROW>
            end
            
            % Copia veloce nel blocco globale
            DATASET_TOTALE(current_row : current_row + righe_valide - 1, :) = buffer_episodio(1:righe_valide, :);
            
            current_row = current_row + righe_valide;
            valid_count = valid_count + 1;

            % --- CHECKPOINT PER LO SPLIT DEI DATI (Train/Val/Test) ---
            if valid_count == 160
                indici_split(1) = current_row - 1;
                fprintf('🎯 CHECKPOINT: Indice Train salvato (%d)\n', indici_split(1));
            elseif valid_count == 180
                indici_split(2) = current_row - 1;
                fprintf('🎯 CHECKPOINT: Indice Validation salvato (%d)\n', indici_split(2));
            end
            
            fprintf('   ✅ Ep. Salvato! Distanza min toccata: %.2f m | Transizioni in memoria: %d\n', min_clearance_volo, current_row - 1);
        end
        
    catch ME
        warning('❌ Errore in Estrazione Dati: %s', ME.message);
        disp(ME.stack(1)); % Stampa la riga esatta dell'errore nella funzione
        continue;
    end
end

%% =======================================================================
% 7. TAGLIO E SALVATAGGIO HDF5 (.h5) PER PYTORCH
% ========================================================================
disp('\n====================================================');
disp('💾 Finalizzazione dataset in corso...');

% TAGLIO: Rimuoviamo gli zeri non utilizzati (se abbiamo allocato troppo)
DATASET_FINALE = DATASET_TOTALE(1:current_row - 1, :);

% TRASPOSIZIONE: MATLAB salva column-major, PyTorch legge row-major.
% Questo evita problemi di reshaping in Python.
dataset_transposto = DATASET_FINALE';

if isfile(file_h5_output)
    delete(file_h5_output); 
end

% Creazione e Scrittura nel file H5 in formato Float32 (single)
h5create(file_h5_output, '/transitions', size(dataset_transposto), 'Datatype', 'single');
h5write(file_h5_output, '/transitions', dataset_transposto);

% Salva l'array degli indici di split direttamente nel file HDF5
h5create('file_h5_output', '/split_indices', [1 2]);
h5write('file_h5_output', '/split_indices', indici_split);
disp('💾 Indici di split salvati con successo nel file HDF5!');

fprintf('🎉 Dataset generato con successo!\n');
fprintf('📊 Dimensioni Finali (Righe x Colonne): %d x 536\n', size(DATASET_FINALE, 1));
fprintf('📁 File salvato come: %s\n', file_h5_output);
