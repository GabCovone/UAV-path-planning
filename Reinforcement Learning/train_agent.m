%% Configurazione Iniziale e Variabili
Ts = 0.1;
rng(1);
plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;

%% Preparazione Modello Simulink
mdl = 'SAC_RL_env';
path = strcat(mdl, "/Inner Loop and Plant Model/High-FidelityModel/");
load_system(mdl);

% Disabilita la propagazione del variant subsystem
set_param(strcat(mdl, '/Inner Loop and Plant Model'), 'PropagateVariantConditions', 'off');

set_param(mdl, 'SimulationMode', 'accelerator'); 

mdlWks = get_param(mdl, 'ModelWorkspace');
mdlWks.assignin('Ts', Ts);
mdlWks.assignin('plantModelFi', plantModelFi);
mdlWks.assignin('useHeading', useHeading);
mdlWks.assignin('initialGainsMultiplier', initialGainsMultiplier);

% Caricamento della DEEN Network per i worker
if isfile('deen_network.mat')
    % Carica il file e lo assegna al workspace del modello
    deen_data = load('deen_network.mat');
    fields = fieldnames(deen_data);
    for i = 1:numel(fields)
        mdlWks.assignin(fields{i}, deen_data.(fields{i}));
    end
else
    error('ATTENZIONE: Il file deen_network.mat non si trova nella cartella!');
end

if get_param(strcat(path, "pos_agente To File"), 'Commented') == "off"
    set_param(strcat(path, "pos_agente To File"), 'Commented', 'on');
end
if get_param(strcat(path, "rays_curr To File"), 'Commented') == "off"
    set_param(strcat(path, "rays_curr To File"), 'Commented', 'on');
end

displayBlks = find_system(path, 'SearchDepth', 1, 'IncludeCommented', 'on', 'BlockType', 'Display');
for k = 1:length(displayBlks)
   set_param(displayBlks{k}, 'Commented', 'on');
end

save_system(mdl);

%% Inizializzazione Pool Parallelo
num_workers = 15;
delete(gcp('nocreate'));
cluster = parcluster('local');
cluster.NumWorkers = num_workers;
pool = parpool(cluster, num_workers);

clear cluster pool

%% CONFIGURAZIONE DATA QUEUE E LOGGING SICURO
logPath = fullfile(pwd, 'registro_morti.txt');
logPath_valid = fullfile(pwd, 'registro_morti_validation.txt');

% Inizializza il file di TRAINING (svuotalo e scrivi l'intestazione)
fileID = fopen(logPath, 'w');
if fileID ~= -1
    fprintf(fileID, '========================================================\n');
    fprintf(fileID, 'INIZIO NUOVO ADDESTRAMENTO: %s\n', char(datetime('now')));
    fprintf(fileID, '========================================================\n');
    fprintf(fileID, 'Ep (Worker) | Step | Reward Totale | Motivo Terminazione\n');
    fprintf(fileID, '--------------------------------------------------------\n');
    fclose(fileID);
else
    error('Impossibile creare il file di log per il training.');
end

% Inizializza il file di VALIDATION (svuotalo e scrivi l'intestazione)
fileID_val = fopen(logPath_valid, 'w');
if fileID_val ~= -1
    fprintf(fileID_val, '========================================================\n');
    fprintf(fileID_val, 'INIZIO NUOVO ADDESTRAMENTO (VALIDATION): %s\n', char(datetime('now')));
    fprintf(fileID_val, '========================================================\n');
    fprintf(fileID_val, 'Ep (Worker) | Step | Reward Totale | Motivo Terminazione\n');
    fprintf(fileID_val, '--------------------------------------------------------\n');
    fclose(fileID_val);
else
    error('Impossibile creare il file di log per la validation.');
end

% Crea le DataQueue e definisci le callback per la scrittura
dq_train = parallel.pool.DataQueue;
afterEach(dq_train, @(msg) appendToLog(logPath, msg));

dq_valid = parallel.pool.DataQueue;
afterEach(dq_valid, @(msg) appendToLog(logPath_valid, msg));

%% Configurazione RL e Ambiente
[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit, Ts);

% NOTA: Passiamo ENTRAMBE le code (`dq_train` e `dq_valid`) all'ambiente
env = get_RL_env(obsInfo, actInfo, actLimit, 'train_scenarios_L1.mat', "validation_scenarios_L1.mat", true, dq_train, dq_valid);

env.UseFastRestart = 'on';

%% Configurazione Opzioni di Addestramento
saveAgentFrequency = 300;

trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5000, ...
    'MaxStepsPerEpisode', 3000, ... 
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 10000, ... 
    'SimulationStorageType', "none", ...
    'SaveFileVersion', "-v7", ... 
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', saveAgentFrequency, ...
    'SaveAgentDirectory', fullfile(pwd, 'agenti_salvati'), ...
    'UseParallel', true ...
);

trainOpts.ParallelizationOptions.Mode = "async";

evalOpts = rlCustomEvaluator_fun(@evaluationFcn, "EvaluationFrequency", saveAgentFrequency);

%% Esecuzione Addestramento
disp('Avvio addestramento parallelo con DataQueue (Train e Validation)...');
trainStats = train(agent, env, trainOpts, "Evaluator", evalOpts);

%% Salvataggio Finale
agent.UseExplorationPolicy = 0;
save('trained_agent.mat', 'agent', 'trainStats', '-v7');
disp('Addestramento completato!');

%% --- FUNZIONI LOCALI ---
function appendToLog(file, msg)
    % Questa funzione viene eseguita ESCLUSIVAMENTE dal client principale.
    % Apre il file in modalità 'append' ('a'), scrive il messaggio e chiude.
    fid = fopen(file, 'a');
    if fid ~= -1
        fprintf(fid, '%s\n', msg);
        fclose(fid);
    end
end