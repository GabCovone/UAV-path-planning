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

displayBlks = find_system(path,'SearchDepth',1,'IncludeCommented', 'on','BlockType','Display');

for k = 1:length(displayBlks)
   set_param(displayBlks{k}, 'Commented','off');
end

save_system('SAC_RL_env')

rng(2);
Ts = 0.1;
assignin('base', 'Ts', Ts);

path_DB_scenari = 'testing_scenarios.mat';
% Usa il percorso assoluto per garantire che MATLAB e Simulink scrivano nello stesso posto
file_registro = fullfile(pwd, 'registro_morti.txt'); 


%%

% 1. Carica l'ambiente
[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();

env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, true, fullfile(pwd, 'registro_morti.txt'));

agent_name = 'saved_agent'; % in genere agent, certe volte è saved_agent

% 2. Carica l'agente salvato
%load('versioni_agenti/agente_v12_rewardexpscaling_816.mat', agent_name);
load('versioni_agenti/agente_v15_lv1_v1.5.mat', agent_name);

% 3. Definisci le opzioni di simulazione
% Vogliamo fargli fare 1 solo episodio, con un massimo di 5500 step (es. 50 secondi a 10Hz)
simOpts = rlSimulationOptions('MaxSteps', 5500, 'NumSimulations', 1);

% 4. Avvia il test!
disp('Avvio simulazione di test...');
experience = sim(env, eval(agent_name), simOpts);

% 5. Estrai e stampa i risultati
%reward_totale = sum(experience.Reward);
%step_totali = experience.Reward.TimeInfo.Length;
reward_totale = sum(experience.Reward.Data);
step_totali = length(experience.Reward.Data); % Più sicuro di navigare in TimeInfo

disp('Test completato.');
disp(['Step sopravvissuti: ', num2str(step_totali)]);
disp(['Reward totale ottenuto: ', num2str(reward_totale)]);

%% --- 6. Recupero Telemetria Diretto (Ricerca Globale) ---

load('sim_pos_agente_1.mat');
    
if ~isempty(sim_pos_agente)
    
    
    sim_pos_agente  = getsamples(sim_pos_agente, 2:sim_pos_agente.Length);   % keep all times except the first

    %experience.Reward.Time  = experience.Reward.Time(2:end,:); % keep all data rows except the first

    % Prende il primo risultato trovato
    dati_pos = sim_pos_agente.Data;
    vettore_tempi = experience.Reward.Time;
    
    % Gestione formato [1 x 3 x N] o [N x 3]
    if size(dati_pos, 2) ~= 3
        dati_pos = squeeze(dati_pos)'; 
    end
    
    disp('✅ Telemetria recuperata con successo dalla gerarchia!');
    graphic_func(path_DB_scenari, scenario_corrente, dati_pos, vettore_tempi); 
else
    disp('❌ Errore: "log_posizione" non trovato in nessuna sottocartella.');
end