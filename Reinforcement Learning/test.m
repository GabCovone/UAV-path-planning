rng(2);

plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;

Ts = 0.1; % Tempo di campionamento (10 Hz)
assignin('base', 'Ts', Ts);

path_DB_scenari = 'training_scenarios.mat';

% 1. Carica l'ambiente
[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
env = get_RL_env(obsInfo, actInfo, path_DB_scenari);

% 2. Carica l'agente salvato
load('agente_v6.mat', 'agent');

% 3. Definisci le opzioni di simulazione
% Vogliamo fargli fare 1 solo episodio, con un massimo di 5500 step (es. 50 secondi a 10Hz)
simOpts = rlSimulationOptions('MaxSteps', 5500, 'NumSimulations', 1);

% 4. Avvia il test!
disp('Avvio simulazione di test...');
experience = sim(env, agent, simOpts);

% 5. Estrai e stampa i risultati
%reward_totale = sum(experience.Reward);
%step_totali = experience.Reward.TimeInfo.Length;
reward_totale = sum(experience.Reward.Data);
step_totali = length(experience.Reward.Data); % Più sicuro di navigare in TimeInfo

disp('Test completato.');
disp(['Step sopravvissuti: ', num2str(step_totali)]);
disp(['Reward totale ottenuto: ', num2str(reward_totale)]);

%% --- 6. Recupero Telemetria Diretto (Ricerca Globale) ---
% dà errore
load('sim_pos_agente_1.mat');
    
if ~isempty(sim_pos_agente)
    % Prende il primo risultato trovato
    dati_pos = sim_pos_agente.Data;
    
    % Gestione formato [1 x 3 x N] o [N x 3]
    if size(dati_pos, 2) ~= 3
        dati_pos = squeeze(dati_pos)'; 
    end
    
    disp('✅ Telemetria recuperata con successo dalla gerarchia!');
    graphic_func(dati_pos, scenario_corrente, path_DB_scenari); 
else
    disp('❌ Errore: "log_posizione" non trovato in nessuna sottocartella.');
end