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

% --- 6. Recupero Telemetria Diretto (Ricerca Globale) ---
% dà errore
if exist('logsout', 'var')
    % Cerca 'log_posizione' OVUNQUE nella gerarchia del modello
    elementi_trovati = logsout.find('Name', 'log_posizione');
    
    if ~isempty(elementi_trovati)
        % Prende il primo risultato trovato
        dati_pos = elementi_trovati{1}.Values.Data;
        
        % Gestione formato [1 x 3 x N] o [N x 3]
        if size(dati_pos, 2) ~= 3
            dati_pos = squeeze(dati_pos)'; 
        end
        
        disp('✅ Telemetria recuperata con successo dalla gerarchia!');
        graphic_func(dati_pos); 
    else
        disp('❌ Errore: "log_posizione" non trovato in nessuna sottocartella.');
    end
else
    disp('❌ Errore: "logsout" non esiste nel workspace.');
gitend