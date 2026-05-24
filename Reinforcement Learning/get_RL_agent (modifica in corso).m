function agent = get_RL_agent(obsInfo, actInfo, StructNumObs, numAct, actLimit, Ts)
    %% 1. DEFINIZIONE DELLE RETI NEURALI
    % Dimensioni dei layer nascosti
    hiddenLayerSize = 128;

    numStateVars = StructNumObs.numState + StructNumObs.numErrors;
    
    numRays = StructNumObs.numRays;

    % --- CRITIC NETWORKS (Q-Values: [Obs, Act] -> Q) ---

    % Ramo 1: Rays (Fully Connected per vettori impilati)
    criticRaysPath = [
        featureInputLayer(numRays, 'Normalization', 'none', 'Name', 'rays_input')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticRaysFC1')
        reluLayer('Name', 'CriticRaysRelu')
        ];
    
    criticStatePath = [
        featureInputLayer(numStateVars, 'Normalization', 'none', 'Name', 'state_input')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticStateFC1')
        reluLayer('Name', 'CriticStateRelu')
        ];
    
    % Ramo Azione
    actionPath = [
        featureInputLayer(numAct, 'Normalization', 'none', 'Name', 'action')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticActionFC1')
        reluLayer('Name', 'CriticActionRelu')
        ];
    
    % Ramo Comune (Osservazioni + Azioni -> Q-Value)
    criticCommonPath = [
        concatenationLayer(1, 3, 'Name', 'CriticCommonConcat')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticCommonFC1')
        reluLayer('Name', 'CriticCommonRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticCommonFC2')
        reluLayer('Name', 'CriticCommonRelu2')
        fullyConnectedLayer(1, 'Name', 'QValue')
        ];
    
    criticNetwork = layerGraph(criticRaysPath);
    criticNetwork = addLayers(criticNetwork, criticStatePath);
    criticNetwork = addLayers(criticNetwork, actionPath);
    criticNetwork = addLayers(criticNetwork, criticCommonPath);
    
    criticNetwork = connectLayers(criticNetwork, 'CriticRaysRelu/out', 'CriticCommonConcat/in1');
    criticNetwork = connectLayers(criticNetwork, 'CriticStateRelu/out', 'CriticCommonConcat/in2');
    criticNetwork = connectLayers(criticNetwork, 'CriticActionRelu/out', 'CriticCommonConcat/in3');
    
    
    % Inizializza due Critic identici
    criticOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    critic1 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    critic2 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    
    % --- ACTOR NETWORK (Policy: Obs -> Action Mean & Std) ---
    % SAC utilizza una policy gaussiana, quindi l'attore deve fornire media e deviazione standard

    actorRaysPath = [
        featureInputLayer(numRays, 'Normalization', 'none', 'Name', 'rays_input')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'ActorRaysFC1')
        reluLayer('Name', 'ActorRaysRelu')
        ];
    
    actorStatePath = [
        featureInputLayer(numStateVars, 'Normalization', 'none', 'Name', 'state_input')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'ActorStateFC1')
        reluLayer('Name', 'ActorStateRelu')
        ];

    actorCommonPath = [
        concatenationLayer(1, 2, 'Name', 'ActorCommonConcat')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC1')
        reluLayer('Name', 'ActorRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        reluLayer('Name', 'ActorRelu2')
        ];
    
    % Ramo della Media (Mean)
    meanPath = [
        fullyConnectedLayer(numAct, 'Name', 'MeanFC2')
        ];
    
    % Ramo della Deviazione Standard (StdDev) - Valori positivi (Softplus)
    stdPath = [
        fullyConnectedLayer(numAct, 'Name', 'StdFC')
        softplusLayer('Name', 'StdSoftplus') 
        ];
    
    actorGraph = layerGraph(actorRaysPath);
    actorGraph = addLayers(actorGraph, actorStatePath);
    actorGraph = addLayers(actorGraph, actorCommonPath);
    actorGraph = addLayers(actorGraph, meanPath);
    actorGraph = addLayers(actorGraph, stdPath);
    
    actorGraph = connectLayers(actorGraph, 'ActorRaysRelu', 'ActorCommonConcat/in1');
    actorGraph = connectLayers(actorGraph, 'ActorStateRelu', 'ActorCommonConcat/in2');
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'MeanFC1/in');
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'StdFC/in');
    % prec 1e-4
    actorOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    actor = rlContinuousGaussianActor(dlnetwork(actorGraph), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', ...
        'ActionMeanOutputNames', 'MeanFC2', ...
        'ActionStandardDeviationOutputNames', 'StdSoftplus');
    
    disp('✅ Reti critics e actor create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE SAC
    % Opzioni specifiche dell'agente

    entropyOptions = rl.option.EntropyWeightOptions(...
        'EntropyWeight', 1, ...
        'LearnRate', 3e-4, ...
        'TargetEntropy', -numAct, ...
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