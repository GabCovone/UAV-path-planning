function env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, path_DB_scenari_eval, logging, logPath, logPathValid)
function env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, logging, logPath)
    
    if nargin < 4, path_DB_scenari = 'training_scenarios.mat'; end
    if nargin < 5, path_DB_scenari_eval = 'validation_scenarios.mat'; end
    
    % Assegnazione nel workspace dei path di scenari di training e
    % validation
    assignin('base', 'path_DB_scenari', path_DB_scenari);
    assignin('base', 'path_DB_scenari_eval', path_DB_scenari_eval);
    
    % Logging
    if nargin < 6, logging = false; end
    if nargin < 7
        logPath = fullfile(pwd, 'registro_morti.txt');
    end
    if nargin < 8
        logPathValid = fullfile(pwd, 'registro_morti_validation.txt');
    end
    if nargin < 5, logging = false; logPath = fullfile(pwd, 'registro_morti.txt'); end
    assignin('base', 'logging', logging);
    logPath_padded = sprintf('%-250s', logPath);
    assignin('base', 'logPath_num', int8(logPath_padded));
    logPathValid_padded = sprintf('%-250s', logPathValid);
    assignin('base', 'logPathValid_num', int8(logPathValid_padded))

    mdl = 'SAC_RL_env';
    agentBlk = [mdl, '/Inner Loop and Plant Model/High-FidelityModel/RL Agent'];

    % Assegnazione nel workspace dei limiti per la normalizzazione
    assignin('base', 'max_delta', actLimit);

    max_deviazione_pos = 100; % Deviazione massima in metri consentita per l'agente
    assignin('base', 'max_deviazione_pos', max_deviazione_pos);
    
    max_deviazione_vel = 20; % Deviazione massima in m/s per la velocità
    assignin('base', 'max_deviazione_vel', max_deviazione_vel);

    max_vel = 30; % Massima velocità lineare
    max_angular_vel = double(pi); % Massima velocità angolare
    assignin('base', 'max_vel', max_vel);
    assignin('base', 'max_angular_vel', max_angular_vel);
    
    tolleranza_goal = 20; % di base 2, per curriculum 20
    assignin('base', 'tolleranza_goal', tolleranza_goal);
    
    % Creazione dell'ambiente Simulink
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);

    assignin('base',"is_validation",false);
    
    % Assegnazione all'ambiente della funzione di reset
    env.ResetFcn = @(in) localResetFcn(in);
    env.ResetFcn = @(in) localResetFcn(in, path_DB_scenari);

    disp('✅ Ambiente RL Simulink creato con successo');
end

function in = localResetFcn(in)
function in = localResetFcn(in, path_DB_scenari)
    % Dichiarazione variabili persistenti
    persistent DB_scenari DB_scenari_eval scenario_corrente episodi
    persistent path_DB_scenari_persistent path_DB_scenari_eval_persistent

    is_validation = evalin('base', 'is_validation');

    path_DB_scenari = evalin('base', 'path_DB_scenari');
    persistent DB_scenari scenario_corrente episodi
    persistent path_DB_persistent
    
    % --- 1. Inizializzazione ad inizio training o se cambia il file del DB ---
    if isempty(DB_scenari) || ~strcmp(path_DB_scenari, path_DB_scenari_persistent)
        disp("Inizializzazione DB scenari...")
        % Carica il file .mat pre-calcolato una volta sola
        path_DB_scenari_persistent = path_DB_scenari;
        data = load(path_DB_scenari_persistent); 
    % --- 1. Inizializzazione o cambio DB ---
    % Carica il database degli scenari se è la prima esecuzione o se il file è cambiato.
    if isempty(DB_scenari) || ~strcmp(path_DB_scenari, path_DB_persistent)
        fprintf('--- ResetFcn: Inizializzazione o cambio DB scenari: "%s" ---\n', path_DB_scenari);
        path_DB_persistent = path_DB_scenari;
        data = load(path_DB_persistent); 
        DB_scenari = data.scenari; 
        
        episodi = 0;
        
        % Inizializzazione del primo scenario
        try
            forced_idx = evalin('base', 'eval_scenario_idx');
            if ~isempty(forced_idx)
                scenario_corrente = forced_idx;
            else
                scenario_corrente = randi(length(DB_scenari));
            end
        catch
            scenario_corrente = randi(length(DB_scenari));
        end
        episodi = 0; % Resetta il contatore episodi solo quando il DB cambia
    end
<<<<<<< HEAD
    
    % --- 2. Si valuta se cambiare scenario (durante il training normale) ---
    % Verifica se stiamo forzando l'indice (Testing)
    % --- 2. Selezione dello scenario (Logica Training vs. Testing) ---
    is_testing_mode = false;
    try
        % Controlla se esiste una variabile per forzare lo scenario (modalità testing/evaluation)
        forced_idx = evalin('base', 'eval_scenario_idx');
        is_testing = ~isempty(forced_idx);
        % Se la variabile esiste ed è valida, la usiamo (modalità testing)
        scenario_corrente = forced_idx;
        disp(['Modalità Testing: Scenario forzato a ', num2str(scenario_corrente)]);
        
        % Verifica che l'indice sia un numero valido e nel range del DB
        if isnumeric(forced_idx) && isscalar(forced_idx) && forced_idx > 0 && forced_idx <= length(DB_scenari)
            scenario_corrente = forced_idx;
            is_testing_mode = true;
            disp(['--- ResetFcn (Testing): Scenario forzato a ', num2str(scenario_corrente), ' ---']);
        end
    catch
        is_testing = false;
        % La variabile 'eval_scenario_idx' non esiste, quindi siamo in modalità training.
        % Non c'è bisogno di fare nulla qui.
    end
    
    if is_testing
        % Se siamo in modalità Test, aggiorniamo SEMPRE lo scenario 
        % con quello imposto dal main script, ignorando il random
        scenario_corrente = evalin('base', 'eval_scenario_idx');
    else
        % Se siamo in Training, procediamo con il cambio casuale
        disp("Cambio casuale di scenario.")
        % Altrimenti, siamo in modalità training e scegliamo a caso
    % Se non siamo in modalità testing, scegli uno scenario casuale per il training
    if ~is_testing_mode
        scenario_corrente = randi(length(DB_scenari));
        disp(['Modalità Training: Cambio casuale allo scenario ', num2str(scenario_corrente)]);
=======

    if isempty(DB_scenari_eval)
        disp("Inizializzazione DB scenari di validation...")
        % Carica il file .mat pre-calcolato una volta sola
        path_DB_scenari_eval = evalin('base', 'path_DB_scenari_eval');
        path_DB_scenari_eval_persistent = path_DB_scenari_eval;
        data = load(path_DB_scenari_eval_persistent); 
        DB_scenari_eval = data.scenari; 
>>>>>>> c3d84b118727a2ffa900aeefbe2503122c540a90
        disp(['--- ResetFcn (Training): Cambio casuale allo scenario ', num2str(scenario_corrente), ' ---']);
    end

    if is_validation
        disp("Cambio casuale di scenario di validation.")
        scenario_corrente = randi(length(DB_scenari_eval));
        scenario = DB_scenari_eval(scenario_corrente);
    end
    % Aggiornamento contatore degli episodi
    episodi = episodi + 1;
    
    if ~is_validation
        % --- 2. Si valuta se cambiare scenario (durante il training normale) ---
        % Verifica se stiamo forzando l'indice (Testing)
        try
            forced_idx = evalin('base', 'eval_scenario_idx');
            is_testing = ~isempty(forced_idx);
        catch
            is_testing = false;
        end
        
        if is_testing
            % Se siamo in modalità Test, aggiorniamo SEMPRE lo scenario 
            % con quello imposto dal main script, ignorando il random
            scenario_corrente = evalin('base', 'eval_scenario_idx');
        else
            % Se siamo in Training, procediamo con il cambio casuale
            disp("Cambio casuale di scenario.")
            scenario_corrente = randi(length(DB_scenari));
        end
    % Si estraggono i dati dello scenario da usare in questo episodio
    scenario = DB_scenari(scenario_corrente);
    
        % Aggiornamento contatore degli episodi
        episodi = episodi + 1;
        
        % Si estraggono i dati dello scenario da usare in questo episodio
        scenario = DB_scenari(scenario_corrente);
    end
    
    % --- Resto della funzione inalterato ---
    % Usa la posizione esatta
    initial_pos = scenario.map.q_start; % è 1 x 3, a differenza di velocità e orientamento
    init_vel = [0; 0; 0]; % Parti da fermo
    init_euler = [0; 0; 0]; % Parti in hovering perfetto
    % --- 3. Assegnazione variabili per la simulazione ---
    initial_pos = scenario.map.q_start;
    init_vel = [0; 0; 0];
    init_euler = [0; 0; 0];

    % Calcolo ingombro della città
    bounds.x_min = squeeze(min(scenario.map.v(:,1,:))); bounds.x_max = squeeze(max(scenario.map.v(:,1,:)));
    bounds.y_min = squeeze(min(scenario.map.v(:,2,:))); bounds.y_max = squeeze(max(scenario.map.v(:,2,:)));
    bounds.z_min = squeeze(min(scenario.map.v(:,3,:))); bounds.z_max = squeeze(max(scenario.map.v(:,3,:)));
    bounds.x_min = min(scenario.map.v(:,1,:), [], 'all'); bounds.x_max = max(scenario.map.v(:,1,:), [], 'all');
    bounds.y_min = min(scenario.map.v(:,2,:), [], 'all'); bounds.y_max = max(scenario.map.v(:,2,:), [], 'all');
    bounds.z_min = min(scenario.map.v(:,3,:), [], 'all'); bounds.z_max = max(scenario.map.v(:,3,:), [], 'all');

    % Assegnazione variabili nel workspace
    % Assegnazione variabili nel workspace di base
    assignin('base', 'init_pos', initial_pos);
    assignin('base', 'init_vel', init_vel);
    assignin('base', 'init_euler', init_euler);
    assignin('base', 'sim_pos_des', scenario.sim_pos_des);
    assignin('base', 'sim_vel_des', scenario.sim_vel_des);
    assignin('base', 'sim_yaw_des', scenario.sim_yaw_des);
    assignin('base', 'pos_goal', scenario.map.q_goal);
    assignin('base', 'bounds', bounds);
    assignin('base', 'dyn_obs', scenario.dynamic_obstacles);

    % IMPORTANTISSIMO: questa variabile ora sarà 'i' durante il loop di test!
    assignin('base', 'scenario_corrente', scenario_corrente);
    
    disp(['✅ Punto spawn drone: [', num2str(initial_pos(:)'), '], Goal a [', num2str(scenario.map.q_goal(:)'),']']);

    fprintf('✅ Reset completato. Spawn a [%.1f, %.1f, %.1f], Goal a [%.1f, %.1f, %.1f]\n', ...
            initial_pos, scenario.map.q_goal);
end