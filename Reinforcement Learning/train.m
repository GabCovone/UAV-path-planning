plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;

path = "SAC_RL_env/Inner Loop and Plant Model/High-FidelityModel/";

if get_param(strcat(path, "pos_agente To File"), 'Commented') == "off"
    set_param(strcat(path, "pos_agente To File"), 'Commented', 'on');
end

Ts = 0.1; % Tempo di campionamento (10 Hz)
assignin('base', 'Ts', Ts);

[obsInfo, actInfo, numObs, numAct, actLimit, StructNumObs] = get_obsInfo_actInfo();

agent = get_RL_agent(obsInfo, actInfo, numAct, actLimit, Ts, StructNumObs);

env = get_RL_env(obsInfo, actInfo, 'training_scenarios.mat', true, fullfile(pwd, 'registro_morti.txt'));

delete(gcp('nocreate'))
pool = parpool(6);


%% Training
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5500, ...
    'MaxStepsPerEpisode', 5500, ... % orig era 1000
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 1000, ... % Determine this based on your reward scaling
    'SimulationStorageType', "none", ...
    'SaveFileVersion', "-v7.3", ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 300, ...
    'SaveAgentDirectory', fullfile(pwd, 'agenti_salvati') ...
);
trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode = "async";

logging = true; 
logPath = fullfile(pwd, 'registro_morti.txt');
assignin('base', 'logging', logging);
logPath_padded = sprintf('%-250s', logPath);
assignin('base', 'logPath_num', double(logPath_padded));
fileID = fopen(logPath, 'w');
if fileID ~= -1
    % Scriviamo un'intestazione pulita per l'inizio del training
    fprintf(fileID, '========================================================\n');
    fprintf(fileID, 'INIZIO NUOVO ADDESTRAMENTO: %s\n', char(datetime('now')));
    fprintf(fileID, '========================================================\n');
    fprintf(fileID, 'Ep (Worker) | Step | Reward Totale | Motivo Terminazione\n');
    fprintf(fileID, '--------------------------------------------------------\n');
    fclose(fileID);
    disp('File di log inizializzato con successo.');
else
    error('Impossibile creare il file di log. Controlla i permessi della cartella.');
end

%env.UseFastRestart = 'on';

%load('deen_network.mat', 'deen_net');

trainStats = train(agent, env, trainOpts);

