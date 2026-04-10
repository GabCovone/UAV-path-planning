% È necessario che:
% - Sia stata creata una traiettoria con il path planning
% - Sia stato eseguito init_simulation.m per configurare Simulink
% - Sia presente un modello DEEN preaddestrato nel path

% load('deen_addestrato.onnx'); 
% disp('Modello DEEN caricato con successo.');

% temp, le mappe andranno prese dagli scenari ad ogni reset
% load('mappa_urbana.mat', 'v', 'n_collision'); 
% 
% disp('🏢 Pre-calcolo dell''ingombro della città...');
% bounds.x_min = squeeze(min(v(:,1,:))); bounds.x_max = squeeze(max(v(:,1,:)));
% bounds.y_min = squeeze(min(v(:,2,:))); bounds.y_max = squeeze(max(v(:,2,:)));
% bounds.z_min = squeeze(min(v(:,3,:))); bounds.z_max = squeeze(max(v(:,3,:)));

plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15; 

% Importazione del modello ONNX convertendolo in un oggetto dlnetwork di MATLAB
deen_net_uninit = importNetworkFromONNX('deen_standalone.onnx');

% - 2024 sono le feature (stato, velocità, voxel, ecc.)
% - 1 è la dimensione del batch (Simulink processa 1 step alla volta)
% - 'BC' sta per C = Channel (Features), B = Batch
dummy_input = dlarray(zeros(2024, 1, 'single'), 'CB');

% 3. Inizializza la rete
deen_net = initialize(deen_net_uninit, dummy_input);

% 4. Salva la rete pronta all'uso!
save('deen_network.mat', 'deen_net');

disp('✅ Rete DEEN inizializzata e salvata con successo!');

% Check rete DEEN

% Creiamo uno stato puramente casuale (rumore)
stato_casuale = rand(1, 2024, 'single');
input_formattato = dlarray(stato_casuale, 'BC');

% Facciamo una predizione
energia_test = predict(deen_net, input_formattato);
disp('Output della rete DEEN:');
disp(extractdata(energia_test));


%% 1. DEFINIZIONE DELL'AMBIENTE SIMULINK
mdl = 'SAC_RL_env';
agentBlk = [mdl, '/Inner Loop and Plant Model/High-FidelityModel/RL Agent']; % Assicurati che il nome del blocco coincida

% Definizione costanti del problema
numVoxels = 1000;
numState = 10; % 3 vel + 3 omega + 4 quat
numNominal = 7; % 3 pos + 3 vel + 1 yaw
numObs = numState + numVoxels + numNominal; % 10 stato + 1000 voxels + 7 nominal = 1017
numAct = 7; % 3 per posizione, 3 per velocità, 1 per lo yaw
Ts = 0.1; % Tempo di campionamento (10 Hz)

% Spazio delle Osservazioni (Observation Space)
obsInfo = rlNumericSpec([1 numObs]);
obsInfo.Name = 'UAV_Observations';

% Spazio delle Azioni (Action Space) :
% Deviazioni (Delta pos, Delta vel, Delta yaw)

% Limiti massimi sulle deviazioni
max_delta_pos = 2.0;  % +/- 2 metri
max_delta_vel = 1.0;  % +/- 1 m/s
max_delta_yaw = 0.5;  % +/- 0.5 rad

actLimit = [max_delta_pos*ones(3,1); max_delta_vel*ones(3,1); max_delta_yaw];
actInfo = rlNumericSpec([1 numAct], ...
    'LowerLimit', -actLimit', ...
    'UpperLimit', actLimit');
actInfo.Name = 'Residual_Actions';

% Creazione dell'ambiente Simulink
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

function in = localResetFcn(in)
    % Dichiarazione variabili persistenti
    persistent DB_scenari scenario_corrente tentativi_attuali max_tentativi
    
    % Inizializzazione ad inizio training
    if isempty(DB_scenari)
        % Carica il file .mat pre-calcolato una volta sola
        data = load('training_scenarios.mat'); 
        DB_scenari = data.scenari; 
        
        scenario_corrente = randi(length(DB_scenari)); % Primo scenario casuale
        tentativi_attuali = 0;
        max_tentativi = 10; % Quante volte si può riprovare la stessa mappa
    end
    
    % Si valuta se cambiare scenario
    if tentativi_attuali >= max_tentativi
        % Si sceglie un nuovo scenario casuale
        disp("Superato il max numero di tentativi per lo scenario corrente. Cambio di scenario.")
        scenario_corrente = randi(length(DB_scenari));
        tentativi_attuali = 0; % Resetta il contatore
    end

    % Aggiornamento contatore dei tentativi
    tentativi_attuali = tentativi_attuali + 1;
    
    % Si estraggono i dati dello scenario da usare in questo episodio
    scenario = DB_scenari(scenario_corrente);
    
    % Domain randomization, in forma di rumore sulla partenza
    x0 = scenario.map.q_start(1) + (rand() - 0.5) * 0.5; % +/- 25 cm
    y0 = scenario.map.q_start(2) + (rand() - 0.5) * 0.5;
    z0 = scenario.map.q_start(3);
    initial_pos = [x0; y0; z0];
    init_vel = [(rand()-0.5)*1.0; (rand()-0.5)*1.0; 0]; % +/- 0.5 m/s
    init_euler = [(rand()-0.5)*0.2; (rand()-0.5)*0.2; 0]; % Roll e Pitch non nulli

    % Set punto di spawn del drone
    dict = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');
    sect = getSection(dict, 'Design Data');
    entry = getEntry(sect, 'initialConditions');
    initStruct = getValue(entry);
    initStruct.posNED = cast([initial_pos(1), initial_pos(2), initial_pos(3)], class(initStruct.posNED));
    setValue(entry, initStruct);
    saveChanges(dict);
    disp(['✅ Condizioni iniziali aggiornate nel file SLDD! Il drone spawnerà a (NED): [', num2str(initStruct.posNED), ']']);

    % Calcolo ingombro della città
    bounds.x_min = squeeze(min(scenario.map.v(:,1,:))); bounds.x_max = squeeze(max(scenario.map.v(:,1,:)));
    bounds.y_min = squeeze(min(scenario.map.v(:,2,:))); bounds.y_max = squeeze(max(scenario.map.v(:,2,:)));
    bounds.z_min = squeeze(min(scenario.map.v(:,3,:))); bounds.z_max = squeeze(max(scenario.map.v(:,3,:)));
    
    % Assegnazione variabili nel workspace
    assignin('base', 'init_pos', initial_pos);
    assignin('base', 'init_vel', init_vel);
    assignin('base', 'init_euler', init_euler);
    assignin('base', 'sim_pos_des', scenario.sim_pos_des);
    assignin('base', 'sim_vel_des', scenario.sim_vel_des);
    assignin('base', 'sim_yaw_des', scenario.sim_yaw_des);
    assignin('base', 'pos_goal', scenario.map.q_goal);
    %assignin('base', 'v', scenario.map.v);
    assignin('base', 'bounds', bounds);
    assignin('base', 'dyn_obs', scenario.dynamic_obstacles);
end

% Assegnazione all'ambiente della funzione di reset
env.ResetFcn = @(in) localResetFcn(in);

disp('✅ Ambiente Simulink creato con successo');

%% 2. DEFINIZIONE DELLE RETI NEURALI
% Dimensioni dei layer nascosti
hiddenLayerSize = 256; 

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
    additionLayer(2, 'Name', 'add')
    reluLayer('Name', 'CriticCommonRelu')
    fullyConnectedLayer(hiddenLayerSize/2, 'Name', 'CriticCommonFC1')
    reluLayer('Name', 'CriticCommonRelu2')
    fullyConnectedLayer(1, 'Name', 'QValue')
    ];

criticNetwork = layerGraph(criticNetwork);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, criticCommonPath);
criticNetwork = connectLayers(criticNetwork, 'CriticStateFC2', 'add/in1');
criticNetwork = connectLayers(criticNetwork, 'CriticActionFC1', 'add/in2');

% Inizializza due Critic identici
criticOptions = rlOptimizerOptions('LearnRate', 1e-3, 'GradientThreshold', 1);
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

%% 3. CREAZIONE DELL'AGENTE SAC
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

disp('✅ Ambiente e Agente SAC inizializzati con successo!');

%% Check agente

disp('🔍 Validazione dell''ambiente in corso...');
validateEnvironment(env);
disp('✅ Ambiente validato con successo!');

disp('🔍 Test della Reset Function in corso...');
for i = 1:5
    fprintf('Reset episodio %d...\n', i);
    
    % Questo è il comando ufficiale per resettare l'ambiente RL.
    % Eseguirà internamente la tua ResetFcn.
    InitialObservation = reset(env); 
    
    disp('Reset completato con successo per questo episodio!');
end
disp('✅ Test reset function superato!');


disp('🔍 Test Fast Restart: Caricamento Scenario 1...');

% Si effettua un reset per caricare il primo scenario in memoria
reset(env); 

% 2. Si attiva il Fast Restart sul modello
set_param('SAC_RL_env', 'FastRestart', 'on');

try
    % 3. Si esegue il primo episodio (Questo "congelerà" la memoria)
    disp('Compilazione e avvio Episodio 1...');
    sim('SAC_RL_env', 'StopTime', '0.1'); 
    disp('✅ Episodio 1 completato. Memoria di Fast Restart bloccata.');
    
    % 4. Si effettua un secondo reset (Caricando un nuovo scenario casuale o il successivo)
    disp('Test Fast Restart: Caricamento Scenario 2...');
    reset(env);
    
    % 5. Si esegue il secondo episodio
    disp('Avvio Episodio 2...');
    sim('SAC_RL_env', 'StopTime', '0.1');
    disp('✅ Episodio 2 completato! Il Fast Restart FUNZIONA.');
    
catch ME
    disp('❌ CRASH DEL FAST RESTART ALL''EPISODIO 2!');
    disp('--------------------------------------------------');
    disp('MESSAGGIO DI ERRORE PRINCIPALE:');
    disp(ME.message);
    disp('--------------------------------------------------');
    
    % Si estraggono i dettagli del crash se ci sono
    if ~isempty(ME.cause)
        disp('CAUSE EFFETTIVE:');
        for k=1:length(ME.cause)
            disp(ME.cause{k}.message);
        end
    end
end

% A prescindere dal risultato, si spegne il Fast Restart per pulizia
set_param('SAC_RL_env', 'FastRestart', 'off');


% % Debug isDone:
% 
% % 1. Reset dell'ambiente (chiama la tua ResetFcn)
% stato_iniziale = reset(env);
% 
% % 2. Ispeziona lo stato iniziale
% disp('Posizione iniziale drone:');
% disp(stato_iniziale); % Controlla se i valori sono sensati
% 
% % 3. Esegui UN SOLO passo di simulazione manualmente
% [prossimo_stato, reward, isDone, info] = step(env, zeros(size(actInfo))); 
% 
% % 4. Analisi del risultato
% fprintf('Reward al passo 0: %f\n', reward);
% fprintf('L''episodio è terminato subito (isDone)? %d\n', isDone);
% 
% if isDone
%     disp('ATTENZIONE: L''ambiente termina istantaneamente!');
%     disp('Controlla la logica di collisione o i limiti del campo nel Plant.');
% end