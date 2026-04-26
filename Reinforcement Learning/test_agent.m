%% Script di Test Batch per Agente SAC (test_agent.m)
clear eval_scenario_idx; clc; close all;

%% 1. Configurazione Iniziale e Percorsi
path = "SAC_RL_env/Inner Loop and Plant Model/High-FidelityModel/";
load_system("SAC_RL_env");

plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;

if get_param(strcat(path, "pos_agente To File"), 'Commented') == "on"
    set_param(strcat(path, "pos_agente To File"), 'Commented', 'off');
end

rng(2);
Ts = 0.1;
assignin('base', 'Ts', Ts);

path_DB_scenari = 'training_scenarios_lv1.mat';
% Usa il percorso assoluto per garantire che MATLAB e Simulink scrivano nello stesso posto
file_registro = fullfile(pwd, 'registro_morti.txt'); 

%% 2. Inizializzazione Ambiente e Agente
disp('Caricamento database scenari...');
data_scenari = load(path_DB_scenari);
num_scenari = length(data_scenari.scenari);

[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, true, file_registro);

agent_name = 'saved_agent'; 
load('versioni_agenti/agente_v15_lv1_v1.mat', agent_name);

simOpts = rlSimulationOptions('MaxSteps', 5500, 'NumSimulations', 1);

% Reset file di log
if isfile(file_registro)
    delete(file_registro);
    disp('File registro resettato con successo.');
end

%% 3. Esecuzione Test Sequenziale
fprintf('\n>>> AVVIO TEST SU %d SCENARI <<<\n', num_scenari);

for i = 1:num_scenari
    fprintf('\n--- Scenario %d/%d ---\n', i, num_scenari);
    
    % Forza l'indice dello scenario
    assignin('base', 'eval_scenario_idx', i);
    
    % Esegue la simulazione
    experience = sim(env, eval(agent_name), simOpts);
    
    reward_totale = sum(experience.Reward.Data);
    step_totali = length(experience.Reward.Data); 
    
    fprintf('Step: %d | Reward: %.2f\n', step_totali, reward_totale);

    % Recupero Telemetria
    if isfile('sim_pos_agente_1.mat')
        load('sim_pos_agente_1.mat');
        if ~isempty(sim_pos_agente)
            sim_pos_agente  = getsamples(sim_pos_agente, 2:sim_pos_agente.Length);   
            dati_pos = sim_pos_agente.Data;
            vettore_tempi = experience.Reward.Time;
            
            if size(dati_pos, 2) ~= 3
                dati_pos = squeeze(dati_pos)'; 
            end
            
            % Disegna il grafico (opzionale: chiudi le vecchie figure per non riempire la RAM)
            % close all; 
            graphic_func(path_DB_scenari, i, dati_pos, vettore_tempi); 
        end
    else
        warning('File telemetria non trovato per lo scenario %d.', i);
    end
end

evalin('base', 'clear eval_scenario_idx');
disp('=== SIMULAZIONI COMPLETATE ===');

%% 4. Analisi Risultati (Parsing del Log)
fprintf('\nAnalisi del registro di sistema in corso...\n');
pause(1); % Assicura che Simulink abbia rilasciato l'handle del file

try
    fid = fopen(file_registro, 'r');
    if fid == -1
        error('Impossibile aprire il file registro in: %s', file_registro);
    end
    testo_registro = fread(fid, '*char')'; 
    fclose(fid);
    
    % Parsing tramite stringhe esatte estratte dal tuo log di Simulink
    num_successi   = count(testo_registro, "TRAGUARDO RAGGIUNTO!");
    num_collisioni = count(testo_registro, "Collisione ostacolo");
    num_schianti   = count(testo_registro, "Schianto a terra");
    num_deviazioni = count(testo_registro, "Deviazione eccessiva");
    num_timeout    = num_scenari - (num_successi + num_collisioni + num_schianti + num_deviazioni);
    
    % Stampa Report Testuale
    fprintf('\n================ REPORT FINALE ================\n');
    fprintf('Scenari Totali: %d\n', num_scenari);
    fprintf('Successi:       %d (%.1f%%)\n', num_successi, (num_successi/num_scenari)*100);
    fprintf('Collisioni:     %d (%.1f%%)\n', num_collisioni, (num_collisioni/num_scenari)*100);
    fprintf('Schianti Terra: %d (%.1f%%)\n', num_schianti, (num_schianti/num_scenari)*100);
    fprintf('Fuori Rotta:    %d (%.1f%%)\n', num_deviazioni, (num_deviazioni/num_scenari)*100);
    fprintf('Timeout:        %d (%.1f%%)\n', num_timeout, (num_timeout/num_scenari)*100);
    fprintf('===============================================\n');
    
    % Generazione Grafico a Torta
    figure('Name', 'Risultati Valutazione', 'NumberTitle', 'off');
    dati_torta = [num_successi, num_collisioni, num_schianti, num_deviazioni, num_timeout];
    etichette = {'Successo', 'Collisione', 'Schianto a Terra', 'Fuori Rotta', 'Timeout'};
    
    idx_validi = dati_torta > 0; 
    if any(idx_validi)
        pie(dati_torta(idx_validi), etichette(idx_validi));
        title(sprintf('Risultati su %d Scenari', num_scenari));
        colormap(gca, [0.4660 0.6740 0.1880; 0.8500 0.3250 0.0980; 0.6350 0.0780 0.1840; 0.9290 0.6940 0.1250; 0.5 0.5 0.5]);
    else
        disp('Nessun dato valido nel registro per generare il grafico a torta.');
    end
    
catch ME
    fprintf('❌ Errore durante l''analisi del file di log:\n%s\n', ME.message);
end