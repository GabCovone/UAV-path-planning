%% Collaudo Macchina Pura con Inizializzazione Ambiente SAC
clc; clear; close all;

% --- 1. Configurazione Scenario e Variabili Base ---
scenario_idx = 16; % Scegli qui lo scenario da testare
path_DB_scenari = 'test_scenarios_L1.mat'; % File del database scenari
Ts = 0.1; % Tempo di campionamento

% Variabili strutturali richieste dal modello Simulink
plantModelFi = 1;            
useHeading = 1;              
tunePositionController = 0;

initialGainsMultiplier = 1.0; 

assignin('base', 'eval_scenario_idx', scenario_idx);
assignin('base', 'Ts', Ts);
assignin('base', 'plantModelFi', plantModelFi);
assignin('base', 'useHeading', useHeading);
assignin('base', 'tunePositionController', tunePositionController);
assignin('base', 'initialGainsMultiplier', initialGainsMultiplier);

% --- 2. Inizializzazione Stile SAC (Il Trucco Magico) ---
disp('⚙️ Inizializzazione ambiente per caricare le condizioni di partenza...');
[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, "validation_scenarios.mat", true, fullfile(pwd, 'registro_morti.txt'));
reset(env); 
disp('✅ Workspace popolato correttamente con i dati dello scenario.');

% --- 3. Preparazione e Caricamento Modello Debugging ---
model_name = 'MultirotorModel_Debugging';
load_system(model_name);

% Imposta un tempo di simulazione lungo
set_param(model_name, 'StopTime', '550'); 

% --- 4. Avvio della Simulazione (Pura, senza l'agente) ---
disp(['🚀 Avvio collaudo PID puro sullo scenario ', num2str(scenario_idx), '...']);
out = sim(model_name);
disp('✅ Volo terminato.');

% --- 5. Estrazione Dati e Grafico 3D ---
disp('Recupero telemetria in corso...');
try
    % Carica i dati salvati su disco dal blocco "To File" di Simulink
    load('sim_pos_agente.mat');
    
    if ~isempty(sim_pos_agente)
        % Rimuove il primo sample anomalo (come nel tuo script originale)
        sim_pos_agente  = getsamples(sim_pos_agente, 2:sim_pos_agente.Length);
        
        dati_pos = sim_pos_agente.Data;
        vettore_tempi = sim_pos_agente.Time;
        
        % Gestione formato matriciale (da [1 x 3 x N] a [N x 3])
        if size(dati_pos, 2) ~= 3
            dati_pos = squeeze(dati_pos)'; 
        end
        
        disp('📊 Generazione del grafico 3D con ostacoli in corso...');
        % Richiama la tua funzione grafica personalizzata
        graphic_func(path_DB_scenari, scenario_idx, dati_pos, vettore_tempi); 
    end
catch
    disp('❌ Errore: File "sim_pos_agente.mat" non trovato nella cartella.');
    disp('Assicurati che il blocco "To File" chiamato "pos_agente To File" in Simulink non sia commentato.');
end