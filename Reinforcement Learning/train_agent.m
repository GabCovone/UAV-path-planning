plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;

%%

evalin('base', 'clear eval_scenario_idx');

path = "SAC_RL_env/Inner Loop and Plant Model/High-FidelityModel/";

load_system("SAC_RL_env");

if get_param(strcat(path, "pos_agente To File"), 'Commented') == "off"
    set_param(strcat(path, "pos_agente To File"), 'Commented', 'on');
end
if get_param(strcat(path, "rays_curr To File"), 'Commented') == "off"
    set_param(strcat(path, "rays_curr To File"), 'Commented', 'on');
end

displayBlks = find_system(path,'SearchDepth',1,'IncludeCommented', 'on','BlockType','Display');

for k = 1:length(displayBlks)
   set_param(displayBlks{k}, 'Commented','on');
end

save_system('SAC_RL_env')

%%

Ts = 0.1; % Tempo di campionamento (10 Hz)
rng(1);
assignin('base', 'Ts', Ts);

[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();

agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit, Ts);

% agent = load('trained_agent_part1.mat'); 
% agent = agent.agent;
% 
% agent.AgentOptions.ActorOptimizerOptions.LearnRate = 5e-5;
% [agent.AgentOptions.CriticOptimizerOptions.LearnRate] = deal(1e-4);

num_workers = 8;

env = get_RL_env(obsInfo, actInfo, actLimit, 'training_scenarios.mat', true, fullfile(pwd, 'registro_morti.txt'));

delete(gcp('nocreate'))
cluster = parcluster('local');
cluster.NumWorkers = num_workers;
pool = parpool(cluster, 8);

%% Training
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 5000, ...
    'MaxStepsPerEpisode', 3000, ... % orig era 1000
    'ScoreAveragingWindowLength', 50, ...
    'StopTrainingCriteria', 'AverageReward', ...
    'StopTrainingValue', 10000, ... % Determine this based on your reward scaling
    'SimulationStorageType', "none", ...
    'SaveFileVersion', "-v7.3", ...
    'SaveAgentCriteria', 'EpisodeFrequency', ...
    'SaveAgentValue', 300, ...
    'SaveAgentDirectory', fullfile(pwd, 'agenti_salvati') ...
);
trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode = "async";

evalOpts = rlEvaluator("EvaluationFrequency",300, "NumEpisodes", 3, "RandomSeeds", 1);

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

trainStats = train(agent, env, trainOpts, "Evaluator", evalOpts);

%% Salva agente

agent.UseExplorationPolicy = 0;
save('trained_agent.mat', 'agent', 'trainStats');