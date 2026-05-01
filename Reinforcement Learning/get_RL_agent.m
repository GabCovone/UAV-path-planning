function agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit, Ts)
    %% 1. DEFINIZIONE DELLE RETI NEURALI
    % Dimensioni dei layer nascosti
    hiddenLayerSize = 256; 
    
    % --- CRITIC NETWORKS (Q-Values: [Obs, Act] -> Q) ---
    criticNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize * 2, 'Name', 'CriticStateFC1')
        swishLayer('Name', 'CriticSwish1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticStateFC2')
        ];
    
    actionPath = [
        featureInputLayer(numAct, 'Normalization', 'none', 'Name', 'action')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticActionFC1')
        ];
    
    criticCommonPath = [
        additionLayer(2, 'Name', 'add')
        swishLayer('Name', 'CriticCommonSwish1')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticCommonFC1')
        swishLayer('Name', 'CriticCommonSwish2')
        fullyConnectedLayer(1, 'Name', 'QValue')
        ];
    
    criticNetwork = layerGraph(criticNetwork);
    criticNetwork = addLayers(criticNetwork, actionPath);
    criticNetwork = addLayers(criticNetwork, criticCommonPath);
    
    criticNetwork = connectLayers(criticNetwork, 'CriticActionFC1/out', 'add/in1'); % <-- Modificato collegamento
    criticNetwork = connectLayers(criticNetwork, 'CriticStateFC2/out', 'add/in2'); % <-- Modificato collegamento
    
    % Inizializza due Critic identici
    criticOptions = rlOptimizerOptions('LearnRate', 5e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    critic1 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    critic2 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', 'ActionInputNames', 'action');
    
    % --- ACTOR NETWORK (Policy: Obs -> Action Mean & Std) ---
    % SAC utilizza una policy gaussiana, quindi l'attore deve fornire media e deviazione standard
    actorNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize * 2, 'Name', 'ActorFC1')
        swishLayer('Name', 'ActorSwish1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        swishLayer('Name', 'ActorSwish2')
        ];
    
    % Ramo della Media (Mean)
    meanPath = [
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'MeanFC1')
        swishLayer('Name', 'MeanSwish')
        fullyConnectedLayer(numAct, 'Name', 'MeanFC2')
        ];
    
    % Ramo della Deviazione Standard (StdDev) - Valori positivi (Softplus)
    stdPath = [
        fullyConnectedLayer(numAct, 'Name', 'StdFC')
        swishLayer('Name', 'StdSwish')
        softplusLayer('Name', 'StdSoftplus') 
        ];
    
    actorGraph = layerGraph(actorNetwork);
    actorGraph = addLayers(actorGraph, meanPath);
    actorGraph = addLayers(actorGraph, stdPath);
    
    actorGraph = connectLayers(actorGraph, 'ActorSwish2', 'MeanFC1/in');
    actorGraph = connectLayers(actorGraph, 'ActorSwish2', 'StdFC/in');
    
    actorOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    actor = rlContinuousGaussianActor(dlnetwork(actorGraph), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation', ...
        'ActionMeanOutputNames', 'MeanFC2', ...
        'ActionStandardDeviationOutputNames', 'StdSoftplus');
    
    disp('✅ Reti critics e actor create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE SAC
    % Opzioni specifiche dell'agente
    
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