function agent = get_RL_agent(obsInfo, actInfo, numAct, actLimit, Ts, StructNumObs)
    %% 0. CALCOLO DIMENSIONI INPUT DA STRUCT
    % Calcola il numero totale di variabili di stato (cinematica + errori)
    numStateVars = StructNumObs.numState + StructNumObs.numErrors;
    
    numRays = StructNumObs.numRays;
    
    %% 1. DEFINIZIONE DELLE RETI NEURALI
    hiddenLayerSize = 256; 
    
    % ==========================================
    % --- CRITIC NETWORKS (Twin Critics) ---
    % ==========================================
    
    % Ramo 1: Rays (Fully Connected per vettori impilati)
    criticRaysPath = [
        featureInputLayer(numRays, 'Normalization', 'none', 'Name', 'rays_input')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticRaysFC1')
        swishLayer('Name', 'CriticRaysSwish1')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticRaysFC2')
        swishLayer('Name', 'CriticRaysSwish2')
        ];
        
    % Ramo 2: Stato Cinematico ed Errori (Fully Connected)
    criticStatePath = [
        featureInputLayer(numStateVars, 'Normalization', 'none', 'Name', 'state_input')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticStateFC1')
        swishLayer('Name', 'CriticStateSwish')
        ];
        
    % Ramo Fusione Osservazioni (Voxel + Stato)
    criticObsConcatPath = [
        concatenationLayer(1, 2, 'Name', 'CriticObsConcat')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticObsFC')
        layerNormalizationLayer('Name', 'CriticObsLN')
        ];
    
    % Ramo Azione
    criticActionPath = [
        featureInputLayer(numAct, 'Normalization', 'none', 'Name', 'action')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'CriticActionFC')
        layerNormalizationLayer('Name', 'CriticActionLN')
        ];
    
    % Ramo Comune (Osservazioni + Azioni -> Q-Value)
    criticCommonPath = [
        additionLayer(2, 'Name', 'CriticAdd')
        swishLayer('Name', 'CriticCommonSwish1')
        fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticCommonFC')
        layerNormalizationLayer('Name', 'CriticCommonLN')
        swishLayer('Name', 'CriticCommonSwish2')
        fullyConnectedLayer(1, 'Name', 'QValue')
        ];
    
    % Assemblaggio Grafo Critic
    criticNetwork = layerGraph();
    criticNetwork = addLayers(criticNetwork, criticRaysPath);
    criticNetwork = addLayers(criticNetwork, criticStatePath);
    criticNetwork = addLayers(criticNetwork, criticObsConcatPath);
    criticNetwork = addLayers(criticNetwork, criticActionPath);
    criticNetwork = addLayers(criticNetwork, criticCommonPath);
    
    % Connessioni Critic
    criticNetwork = connectLayers(criticNetwork, 'CriticRaysSwish2', 'CriticObsConcat/in1');
    criticNetwork = connectLayers(criticNetwork, 'CriticStateSwish', 'CriticObsConcat/in2');
    criticNetwork = connectLayers(criticNetwork, 'CriticObsLN', 'CriticAdd/in1');
    criticNetwork = connectLayers(criticNetwork, 'CriticActionLN', 'CriticAdd/in2');
    
    criticOptions = rlOptimizerOptions('LearnRate', 8e-4, 'GradientThreshold', 10, 'L2RegularizationFactor', 1e-4);
    
    % Nota: ObservationInputNames ora è un cell array con i due rami di input
    critic1 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', {'rays_input', 'state_input'}, 'ActionInputNames', 'action');
    critic2 = rlQValueFunction(dlnetwork(criticNetwork), obsInfo, actInfo, ...
        'ObservationInputNames', {'rays_input', 'state_input'}, 'ActionInputNames', 'action');
    
    % ==========================================
    % --- ACTOR NETWORK (Policy: Obs -> Action) ---
    % ==========================================
    
    % Ramo 1: Rays Actor
    actorRaysPath = [
            featureInputLayer(numRays, 'Normalization', 'none', 'Name', 'rays_input')
            fullyConnectedLayer(256, 'Name', 'ActorRaysFC1')
            swishLayer('Name', 'ActorRaysSwish1')
            fullyConnectedLayer(128, 'Name', 'ActorRaysFC2')
            swishLayer('Name', 'ActorRaysSwish2')
            ];
        
    % Ramo 2: Stato Actor
    actorStatePath = [
        featureInputLayer(numStateVars, 'Normalization', 'none', 'Name', 'state_input')
        fullyConnectedLayer(128, 'Name', 'ActorStateFC')
        swishLayer('Name', 'ActorStateSwish')
        ];
        
    % Ramo Comune Actor (Fusione -> Feature condivise per Media/Std)
    actorCommonPath = [
        concatenationLayer(1, 2, 'Name', 'ActorConcat')
        fullyConnectedLayer(hiddenLayerSize * 2, 'Name', 'ActorFC1')
        layerNormalizationLayer('Name', 'ActorLN1')
        swishLayer('Name', 'ActorSwish1')
        fullyConnectedLayer(hiddenLayerSize, 'Name', 'ActorFC2')
        layerNormalizationLayer('Name', 'ActorLN2')
        swishLayer('Name', 'ActorSwish2')
        ];
    
    % Ramo della Media (Mean)
    meanPath = [
        fullyConnectedLayer(numAct, 'Name', 'MeanFC')
        tanhLayer('Name', 'MeanTanh') 
        scalingLayer('Name', 'MeanScale', 'Scale', actLimit) 
        ];
    
    % Ramo della Deviazione Standard (StdDev)
    stdPath = [
        fullyConnectedLayer(numAct, 'Name', 'StdFC')
        softplusLayer('Name', 'StdSoftplus') 
        ];
    
    % Assemblaggio Grafo Actor
    actorGraph = layerGraph();
    actorGraph = addLayers(actorGraph, actorRaysPath);
    actorGraph = addLayers(actorGraph, actorStatePath);
    actorGraph = addLayers(actorGraph, actorCommonPath);
    actorGraph = addLayers(actorGraph, meanPath);
    actorGraph = addLayers(actorGraph, stdPath);
    
    % Connessioni Actor
    actorGraph = connectLayers(actorGraph, 'ActorRaysSwish2', 'ActorConcat/in1');
    actorGraph = connectLayers(actorGraph, 'ActorStateSwish', 'ActorConcat/in2');
    actorGraph = connectLayers(actorGraph, 'ActorSwish2', 'MeanFC');
    actorGraph = connectLayers(actorGraph, 'ActorSwish2', 'StdFC');
    
    actorOptions = rlOptimizerOptions('LearnRate', 1e-4, 'GradientThreshold', 5, 'L2RegularizationFactor', 1e-4);
    
    % Anche qui, l'actor accetta le due osservazioni separate
    actor = rlContinuousGaussianActor(dlnetwork(actorGraph), obsInfo, actInfo, ...
        'ObservationInputNames', {'rays_input', 'state_input'}, ...
        'ActionMeanOutputNames', 'MeanScale', ...
        'ActionStandardDeviationOutputNames', 'StdSoftplus');
    
    disp('✅ Reti critics e actor create con successo');
    
    %% 2. CREAZIONE DELL'AGENTE SAC
    agentOpts = rlSACAgentOptions(...
        'SampleTime', Ts, ...
        'DiscountFactor', 0.99, ...
        'TargetSmoothFactor', 5e-3, ...
        'ExperienceBufferLength', 1e6, ...
        'MiniBatchSize', 256, ...
        'NumStepsToLookAhead', 1); 
    
    agentOpts.CriticOptimizerOptions = criticOptions;
    agentOpts.ActorOptimizerOptions = actorOptions;
    
    agent = rlSACAgent(actor, [critic1, critic2], agentOpts);
    
    disp('✅ Agente RL SAC creato con successo!');
end