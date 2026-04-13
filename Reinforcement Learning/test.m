% 1. Carica l'ambiente
[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
env = get_RL_env(obsInfo, actInfo);

% 2. Carica l'agente salvato
load('agente_v4.mat', 'agent');

% 3. Definisci le opzioni di simulazione
% Vogliamo fargli fare 1 solo episodio, con un massimo di 500 step (es. 50 secondi a 10Hz)
simOpts = rlSimulationOptions('MaxSteps', 5000, 'NumSimulations', 1);

% 4. Avvia il test!
disp('Avvio simulazione di test...');
experience = sim(env, agent, simOpts);

% 5. Estrai e stampa i risultati
reward_totale = sum(experience.Reward);
step_totali = numel(experience.Reward);

disp(['Test completato.']);
disp(['Step sopravvissuti: ', num2str(step_totali)]);
disp(['Reward totale ottenuto: ', num2str(reward_totale)]);