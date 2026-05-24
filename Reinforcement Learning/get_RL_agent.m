function agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit, Ts)
    %% 1. DEFINIZIONE DELLE RETI NEURALI
    % Dimensioni dei layer nascosti
    hiddenLayerSize = 128; %256
    
    % --- CRITIC NETWORKS (Q-Values: [Obs, Act] -> Q) ---
    criticNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticStateFC1')
        reluLayer('Name', 'CriticRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticStateFC2')
        ];
    
    actionPath = [
        featureInputLayer(numAct, 'Normalization', 'none', 'Name', 'action')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticActionFC1')
        ];
    
    criticCommonPath = [
        concatenationLayer(1, 2, 'Name', 'CriticCommonConcat')
        reluLayer('Name', 'CriticCommonRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticCommonFC1')
        reluLayer('Name', 'CriticCommonRelu2')
        fullyConnectedLayer(1, 'Name', 'QValue')
        ];
    
    criticNetwork = layerGraph(criticNetwork);
    criticNetwork = addLayers(criticNetwork, actionPath);
    criticNetwork = addLayers(criticNetwork, criticCommonPath);
    
    criticNetwork = connectLayers(criticNetwork, 'CriticStateFC2/out', 'CriticCommonConcat/in1');
    criticNetwork = connectLayers(criticNetwork, 'CriticActionFC1/out', 'CriticCommonConcat/in2');
    
    % Inizializza due Critic identici
    criticOptions = rlOptimizerOptions('LearnRate', 5e-3, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    critic1 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    critic2 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    
    % --- ACTOR NETWORK (Policy: Obs -> Action Mean & Std) ---
    % SAC utilizza una policy gaussiana, quindi l'attore deve fornire media e deviazione standard
    actorNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC1')
        reluLayer('Name', 'ActorRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        reluLayer('Name', 'ActorRelu2')
        ];
    
    % Ramo della Media (Mean)
    meanPath = [
        fullyConnectedLayer(numAct, 'Name', 'MeanFC')
        ];
    
    % Ramo della Deviazione Standard (StdDev) - Valori positivi (Softplus)
    stdPath = [
        fullyConnectedLayer(numAct, 'Name', 'StdFC')
        softplusLayer('Name', 'StdSoftplus') 
        ];
    
    actorGraph = layerGraph(actorNetwork);
    actorGraph = addLayers(actorGraph, meanPath);
    actorGraph = addLayers(actorGraph, stdPath);
    
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'MeanFC/in');
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'StdFC/in');

    actorDLNet = dlnetwork(actorGraph);
    
    % Inizializza a zero pesi e bias dell'ultimo layer della media
    % Ottenere i parametri della rete
    pars = actorDLNet.Learnables;
    
    % Trovare gli indici dei parametri di MeanFC2
    meanFC2WeightIdx = find(contains(pars.Layer, "MeanFC") & contains(pars.Parameter, "Weights"));
    meanFC2BiasIdx = find(contains(pars.Layer, "MeanFC") & contains(pars.Parameter, "Bias"));
    
    % Inizializzare pesi e bias a zero
    pars.Value(meanFC2WeightIdx) = {dlarray(zeros(size(pars.Value{meanFC2WeightIdx})))};
    pars.Value(meanFC2BiasIdx) = {dlarray(zeros(size(pars.Value{meanFC2BiasIdx})))};
    
    % Aggiornare la rete con i nuovi parametri
    actorDLNet.Learnables = pars;

    % prec 1e-4
    actorOptions = rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    actor = rlContinuousGaussianActor(actorDLNet, obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', ...
        'ActionMeanOutputNames', 'MeanFC', ...
        'ActionStandardDeviationOutputNames', 'StdSoftplus');
    
    disp('✅ Reti critics e actor create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE SAC
    % Opzioni specifiche dell'agente

    % entropyOptions = rl.option.EntropyWeightOptions(...
    %     'EntropyWeight', 1, ...
    %     'LearnRate', 3e-4, ...
    %     'TargetEntropy', -numAct, ...
    %     'Algorithm', 'adam');

    entropyOptions = rl.option.EntropyWeightOptions(...
        'EntropyWeight', 0.005, ...
        'LearnRate', 1e-12, ...
        'Algorithm', 'adam');
    
    agentOpts = rlSACAgentOptions(...
    'SampleTime', Ts, ...
    'DiscountFactor', 0.99, ...
    'TargetSmoothFactor', 1e-2, ... %5e-3
    'ExperienceBufferLength', 1e6, ...
    'MiniBatchSize', 256, ...
    'EntropyWeightOptions', entropyOptions, ...
    'NumStepsToLookAhead', 1); % Standard per SAC

    % Assegnazione degli ottimizzatori all'interno delle opzioni
    agentOpts.CriticOptimizerOptions = criticOptions;
    agentOpts.ActorOptimizerOptions = actorOptions;
    
    % Creazione dell'agente
    agent = rlSACAgent(actor, [critic1, critic2], agentOpts);
    
    disp('✅ Agente RL SAC creato con successo!');
end