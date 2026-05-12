%% Script per la Generazione Automatica di tutti i Dataset per il Curriculum Learning
% Questo script automatizza la creazione dei set di training, validazione e
% test per tutti e 4 i livelli di difficoltà definiti.
% Versione ottimizzata e refattorizzata.
clear; clc; close all;

disp('--- AVVIO GENERAZIONE COMPLETA DEI DATASET ---');

% -------------------------------------------------------------------------
%                           IMPOSTAZIONI GLOBALI
% -------------------------------------------------------------------------
output_dir = 'Datasets'; % Nome della sottocartella in cui salvare i file

% Numero di scenari per ogni set.
% Livelli complessi (2,3,4) richiedono più varietà.
scenari_config.train      = 200;
scenari_config.validation = 25;
scenari_config.test       = 25;

% Per il livello 1 (più semplice), si possono usare meno scenari.
scenari_config_L1.train      = 80;
scenari_config_L1.validation = 20;
scenari_config_L1.test       = 20;
% -------------------------------------------------------------------------

% Creazione della cartella di output se non esiste
if ~exist(output_dir, 'dir')
   mkdir(output_dir);
   fprintf('Cartella di output creata: %s\n', output_dir);
end

% Definizione dei cicli
livelli = 2; % MODIFICA: Rigenera solo il Livello 2
set_tipi = {'train', 'validation', 'test'};

total_files = length(livelli) * length(set_tipi);
file_count = 0;

% Loop principale
for livello = livelli
    
    % Seleziona la configurazione corretta del numero di scenari
    if livello == 1
        current_config = scenari_config_L1;
    else
        current_config = scenari_config;
    end

    for i = 1:length(set_tipi)
        tipo_set = set_tipi{i};
        num_scenari = current_config.(tipo_set);
        
        file_count = file_count + 1;
        fprintf('\n====================================================\n');
        fprintf('Generazione file %d di %d...\n', file_count, total_files);
        
        % Composizione del nome del file di output
        nome_file = sprintf('%s_scenarios_L%d.mat', tipo_set, livello);
        path_completo = fullfile(output_dir, nome_file);
        
        fprintf('-> Livello: %d\n', livello);
        fprintf('-> Tipo Set: %s\n', tipo_set);
        fprintf('-> N. Scenari: %d\n', num_scenari);
        fprintf('-> File Output: %s\n', path_completo);
        
        % Chiamata alla funzione che contiene la logica di 'crea_scenari.m'
        try
            genera_e_salva_scenari(livello, num_scenari, path_completo);
            fprintf('-> ✅ File salvato con successo.\n');
        catch ME
            fprintf('-> ❌ ERRORE durante la generazione per il livello %d, tipo %s.\n', livello, tipo_set);
            fprintf('   Messaggio di errore: %s\n', ME.message);
        end
    end
end

fprintf('\n====================================================\n');
disp('--- GENERAZIONE COMPLETATA CON SUCCESSO! ---');


%% =======================================================================
%               FUNZIONI LOCALI (Helpers)
% ========================================================================

function genera_e_salva_scenari(livello, num_scenari, nome_file)
    % Funzione orchestratrice che gestisce la creazione di un singolo file.

    % 1. Ottieni la configurazione per il livello richiesto
    config = get_level_config(livello);

    % 2. Genera gli scenari "grezzi" (con traiettorie di diversa lunghezza)
    scenari_grezzi = crea_scenari_grezzi(livello, num_scenari, config.n_collision, ...
        config.x_max, config.y_max, config.z_max, config.dynamic_obs, config.z_threshold);

    % 3. Esegui il padding per uniformare tutti gli scenari
    scenari_padded = pad_scenarios(scenari_grezzi, config.paddingDynObs);

    % 4. Salva il file .mat finale
    scenari = scenari_padded; % Rinomina la variabile per coerenza con il codice che la carica
    save(nome_file, 'scenari', '-v7.3'); % -v7.3 è per file > 2GB
end

function config = get_level_config(livello)
    % Restituisce una struct con tutti i parametri per un dato livello.
    % Questo pulisce il codice e centralizza la configurazione.
    config.paddingDynObs = true;
    
    switch livello
        case 1 % Navigazione Base
            config.x_max = 500; config.y_max = 500; config.z_max = 500;
            config.n_collision = 0; config.num_dyn_obs = 0; config.statici = "no";
            config.min_raggi = 0.0; config.max_raggi = 0.0; config.z_threshold = 4.0;
        case 2 % Evasione Ostacoli Statici
            config.x_max = 1000; config.y_max = 1000; config.z_max = 1000;
            config.n_collision = 40; config.num_dyn_obs = 0; config.statici = "si";
            config.min_raggi = 2.0; config.max_raggi = 2.0; config.z_threshold = 1.5;
        case 3 % Evasione Ostacoli Dinamici (Isolati)
            config.x_max = 1000; config.y_max = 1000; config.z_max = 1000;
            config.n_collision = 5; config.num_dyn_obs = 5; config.statici = "no";
            config.min_raggi = 2.0; config.max_raggi = 5.0; config.z_threshold = 1.5;
        case 4 % Integrazione e Complessità
            config.x_max = 2000; config.y_max = 2000; config.z_max = 1000;
            config.n_collision = 50; config.num_dyn_obs = 10; config.statici = "casuale";
            config.min_raggi = 2.0; config.max_raggi = 5.0; config.z_threshold = 1.5;
        otherwise
            error("Livello non valido. Scegliere un valore tra 1 e 4.");
    end
    
    config.dynamic_obs.numero = config.num_dyn_obs;
    config.dynamic_obs.statici = config.statici;
    config.dynamic_obs.raggi = [config.min_raggi config.max_raggi];
end

function scenari_padded = pad_scenarios(scenari, do_pad_dyn_obs)
    % Esegue il padding su un array di scenari per renderli tutti della
    % stessa lunghezza, necessario per il Fast Restart di Simulink.
    
    scenari_padded = scenari; % Copia per non modificare l'originale

    % 1. Trova la lunghezza massima (MAX_STEPS) in tutto il set di scenari
    if isempty(scenari_padded)
        disp('Nessuno scenario da processare. Salto il padding.');
        return;
    end
    MAX_STEPS = max(arrayfun(@(s) s.sim_pos_des.Length, scenari_padded));
    fprintf('Uniformazione a %d step...\n', MAX_STEPS);

    % 2. Applica il padding a tutti gli scenari
    for i = 1:length(scenari_padded)
        
        % Padding ostacoli dinamici (fino a 10)
        if do_pad_dyn_obs
            num_obs_attuali = length(scenari_padded(i).dynamic_obstacles);
            % BUG FIX: Il ciclo parte da num_obs_attuali + 1 per non
            % sovrascrivere gli ostacoli esistenti.
            for k = (num_obs_attuali + 1):10
                scenari_padded(i).dynamic_obstacles(k).p0 = [0 0 0];
                scenari_padded(i).dynamic_obstacles(k).v = [0 0 0];
                scenari_padded(i).dynamic_obstacles(k).radius = 0;
            end
        end
        
        % Padding delle timeseries di traiettoria
        lunghezza_attuale = scenari_padded(i).sim_pos_des.Length;
        steps_mancanti = MAX_STEPS - lunghezza_attuale;
    
        if steps_mancanti > 0
            % Estrazione dati correnti
            t_corrente    = scenari_padded(i).sim_pos_des.Time;
            pos_corrente  = scenari_padded(i).sim_pos_des.Data;
            vel_corrente  = scenari_padded(i).sim_vel_des.Data;
            yaw_corrente  = scenari_padded(i).sim_yaw_des.Data;
            
            % Padding del Tempo (estrapolazione lineare)
            dt = t_corrente(2) - t_corrente(1);
            t_mancante = (t_corrente(end) + dt : dt : t_corrente(end) + dt * steps_mancanti)';
            t_nuovo = [t_corrente; t_mancante];
            
            % Padding Posizione (Hovering sul traguardo)
            pos_mancante = repmat(pos_corrente(end, :), steps_mancanti, 1);
            pos_nuova = [pos_corrente; pos_mancante];
            
            % Padding Velocità (Fermo a zero)
            vel_mancante = zeros(steps_mancanti, 3);
            vel_nuova = [vel_corrente; vel_mancante];
            
            % Padding Yaw (Mantiene l'ultimo orientamento)
            yaw_mancante = repmat(yaw_corrente(end, :), steps_mancanti, 1);
            yaw_nuova = [yaw_corrente; yaw_mancante];
            
            % Ricostruzione e salvataggio degli oggetti Timeseries
            scenari_padded(i).sim_pos_des = timeseries(pos_nuova, t_nuovo);
            scenari_padded(i).sim_vel_des = timeseries(vel_nuova, t_nuovo);
            scenari_padded(i).sim_yaw_des = timeseries(yaw_nuova, t_nuovo);
        end
    end
    disp('Padding completato.');
end