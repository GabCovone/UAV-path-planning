plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15; 

[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();

env = get_RL_env(obsInfo, actInfo, true, fullfile(pwd, 'registro_morti.txt'));

agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit);

%% Training
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5000, ...
    'MaxStepsPerEpisode', 5000, ... % orig era 1000
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 800, ... % Determine this based on your reward scaling
    'SaveAgentCriteria', 'EpisodeCount', ...
    'SaveAgentValue', 100, ...
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

