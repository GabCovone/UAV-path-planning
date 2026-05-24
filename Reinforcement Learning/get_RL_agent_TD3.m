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
    
    % --- ACTOR NETWORK (Policy: Obs -> Action Deterministica) ---
    actorNetwork = [
        featureInputLayer(numObs, 'Normalization', 'none', 'Name', 'observation')
        fullyConnectedLayer(hiddenLayerSize * 2, 'Name', 'ActorFC1')
        swishLayer('Name', 'ActorSwish1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        swishLayer('Name', 'ActorSwish2')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'ActorFC3')
        swishLayer('Name', 'ActorSwish3')
        fullyConnectedLayer(numAct, 'Name', 'ActorOutput')
        tanhLayer('Name', 'ActorTanh') 
        ];
    
    actorGraph = layerGraph(actorNetwork);
    
    actorOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    
    % Creazione dell'attore continuo deterministico per TD3
    actor = rlContinuousDeterministicActor(dlnetwork(actorGraph), obsInfo, actInfo, ...
        'ObservationInputNames', 'observation');
    
    disp('✅ Reti critics e actor per TD3 create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE TD3
    % Opzioni specifiche dell'agente TD3
    agentOpts = rlTD3AgentOptions(...
        'SampleTime', Ts, ...
        'DiscountFactor', 0.99, ...
        'TargetSmoothFactor', 5e-3, ...
        'ExperienceBufferLength', 1e6, ...
        'MiniBatchSize', 256, ...
        'NumStepsToLookAhead', 1);
    
    agentOpts.PolicyUpdateFrequency = 2;
    
    % ✅ CONFIGURAZIONE DELL'EXPLORATION MODEL

    % agentOpts.ExplorationModel = rl.option.OrnsteinUhlenbeckActionNoise;
    % agentOpts.ExplorationModel.StandardDeviation = 0.1;
    % agentOpts.ExplorationModel.SampleTime = Ts;
    % agentOpts.ExplorationModel.MeanAttractionConstant = 0.15;
    % agentOpts.ExplorationModel.InitialAction = zeros(numAct, 1);

    explorationOpts = rl.option.OrnsteinUhlenbeckActionNoise(...
        'SampleTime', Ts, ...
        'MeanAttractionConstant', 0.15, ...
        'StandardDeviation', 0.1, ...
        'InitialAction', zeros(1, numAct));
    
    agentOpts.ExplorationModel = explorationOpts;
    
    % Assegnazione degli ottimizzatori
    agentOpts.CriticOptimizerOptions = criticOptions;
    agentOpts.ActorOptimizerOptions = actorOptions;
    
    % Creazione dell'agente
    agent = rlTD3Agent(actor, [critic1, critic2], agentOpts);
    
    disp('✅ Agente RL TD3 creato con successo!');
end