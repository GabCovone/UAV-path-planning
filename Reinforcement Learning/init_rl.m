rng(2);

% È necessario che:
% - Sia stata creata una traiettoria con il path planning
% - Sia stato eseguito init_simulation.m per configurare Simulink
% - Sia presente un modello DEEN preaddestrato nel path

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


%% Creazione ambiente e agente

 Ts = 0.1; % Tempo di campionamento (10 Hz)
 assignin('base', 'Ts', Ts);

[obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();

env = get_RL_env(obsInfo, actInfo, 'training_scenarios.mat', true, fullfile(pwd, 'registro_morti.txt'));

agent = get_RL_agent(obsInfo, actInfo, numObs, numAct, actLimit, Ts);

%% Check ambiente e agente

effettuaCheck = true;

if effettuaCheck
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
end