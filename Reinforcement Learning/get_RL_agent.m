function agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit)
    %% 1. DEFINIZIONE DELLE RETI NEURALI
    % Dimensioni dei layer nascosti
    hiddenLayerSize = 256; 
    
    % --- CRITIC NETWORKS (Q-Values: [Obs, Act] -> Q) ---
    criticNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticStateFC1')
        layerNormalizationLayer('Name', 'CriticStateLN1')
        reluLayer('Name', 'CriticRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticStateFC2')
        layerNormalizationLayer('Name', 'CriticStateLN2')
        ];
    
    actionPath = [
        featureInputLayer(numAct, 'Normalization', 'none', 'Name', 'action')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticActionFC1')
        layerNormalizationLayer('Name', 'CriticActionLN1')
        ];
    
    criticCommonPath = [
        additionLayer(2, 'Name', 'add')
        reluLayer('Name', 'CriticCommonRelu')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticCommonFC1')
        layerNormalizationLayer('Name', 'CriticCommonLN1')
        reluLayer('Name', 'CriticCommonRelu2')
        fullyConnectedLayer(1, 'Name', 'QValue')
        ];
    
    criticNetwork = layerGraph(criticNetwork);
    criticNetwork = addLayers(criticNetwork, actionPath);
    criticNetwork = addLayers(criticNetwork, criticCommonPath);
    
    criticNetwork = connectLayers(criticNetwork, 'CriticStateLN2', 'add/in1'); % <-- Modificato collegamento
    criticNetwork = connectLayers(criticNetwork, 'CriticActionLN1', 'add/in2'); % <-- Modificato collegamento
    
    % Inizializza due Critic identici
    criticOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);
    critic1 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    critic2 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    
    % --- ACTOR NETWORK (Policy: Obs -> Action Mean & Std) ---
    % SAC utilizza una policy gaussiana, quindi l'attore deve fornire media e deviazione standard
    actorNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC1')
        layerNormalizationLayer('Name', 'ActorLN1')
        reluLayer('Name', 'ActorRelu1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        layerNormalizationLayer('Name', 'ActorLN2')
        reluLayer('Name', 'ActorRelu2')
        ];
    
    % Ramo della Media (Mean) - Satura ai limiti definiti in actInfo
    meanPath = [
        fullyConnectedLayer(numAct, 'Name', 'MeanFC')
        tanhLayer('Name', 'MeanTanh') % Scala in [-1, 1]
        scalingLayer('Name', 'MeanScale', 'Scale', actLimit) % Riscala sui limiti fisici
        ];
    
    % Ramo della Deviazione Standard (StdDev) - Valori positivi (Softplus)
    stdPath = [
        fullyConnectedLayer(numAct, 'Name', 'StdFC')
        softplusLayer('Name', 'StdSoftplus') 
        ];
    
    actorGraph = layerGraph(actorNetwork);
    actorGraph = addLayers(actorGraph, meanPath);
    actorGraph = addLayers(actorGraph, stdPath);
    
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'MeanFC');
    actorGraph = connectLayers(actorGraph, 'ActorRelu2', 'StdFC');
    
    actorOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 1);
    actor = rlContinuousGaussianActor(dlnetwork(actorGraph), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', ...
        'ActionMeanOutputNames', 'MeanScale', ...
        'ActionStandardDeviationOutputNames', 'StdSoftplus');
    
    disp('✅ Reti critics e actor create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE SAC
    % Opzioni specifiche dell'agente
    
     Ts = 0.1; % Tempo di campionamento (10 Hz)

     assignin('base', 'Ts', Ts);
    
    agentOpts = rlSACAgentOptions(...
        'SampleTime', Ts, ...
        'DiscountFactor', 0.99, ...
        'TargetSmoothFactor', 5e-3, ...
        'ExperienceBufferLength', 1e6, ...
        'MiniBatchSize', 256, ...
        'NumStepsToLookAhead', 1); % Standard per SAC
    
    % Assegnazione degli ottimizzatori all'interno delle opzioni
    agentOpts.CriticOptimizerOptions = criticOptions;
    agentOpts.ActorOptimizerOptions = actorOptions;
    
    % Creazione dell'agente
    agent = rlSACAgent(actor, [critic1, critic2], agentOpts);
    
    disp('✅ Agente RL SAC creato con successo!');
end