function env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, path_DB_scenari_eval, logging, dq_train, dq_valid)
    
    if nargin < 4, path_DB_scenari = 'training_scenarios.mat'; end
    if nargin < 5, path_DB_scenari_eval = 'validation_scenarios.mat'; end
    
    % Assegnazione nel workspace dei path di scenari di training e validation
    assignin('base', 'path_DB_scenari', path_DB_scenari);
    assignin('base', 'path_DB_scenari_eval', path_DB_scenari_eval);
    
    % Logging
    if nargin < 6, logging = false; end
    if nargin < 7
        dq_train = []; % Fallback in caso non esista
    end
    if nargin < 8
        dq_valid = []; % Fallback in caso non esista
    end
    
    assignin('base', 'logging', logging);
    
    % Salviamo gli oggetti coda direttamente nel workspace del worker
    assignin('base', 'dq_train', dq_train);
    assignin('base', 'dq_valid', dq_valid);

    assignin('base', 'logPath_num', zeros(1, 250, 'int8'));
    assignin('base', 'logPathValid_num', zeros(1, 250, 'int8'));
    
    mdl = 'SAC_RL_env';
    agentBlk = [mdl, '/Inner Loop and Plant Model/High-FidelityModel/RL Agent'];
    
    % Assegnazione nel workspace dei limiti
    assignin('base', 'max_delta', actLimit);
    assignin('base', 'max_deviazione_pos', 100); 
    assignin('base', 'max_deviazione_vel', 20); 
    assignin('base', 'max_vel', 30); 
    assignin('base', 'max_angular_vel', double(pi)); 
    assignin('base', 'tolleranza_goal', 2); 
    
    % Creazione dell'ambiente Simulink
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    assignin('base', "is_validation", false);
    
    % Assegnazione all'ambiente della funzione di reset
    env.ResetFcn = @(in) localResetFcn(in);
    disp('✅ Ambiente RL Simulink creato con successo');
end

function in = localResetFcn(in)
    % Dichiarazione variabili persistenti
    persistent DB_scenari DB_scenari_eval scenario_corrente episodi
    persistent path_DB_scenari_persistent path_DB_scenari_eval_persistent
    is_validation = evalin('base', 'is_validation');
    path_DB_scenari = evalin('base', 'path_DB_scenari');
    
    % --- 1. Inizializzazione ad inizio training o se cambia il file del DB ---
    if isempty(DB_scenari) || ~strcmp(path_DB_scenari, path_DB_scenari_persistent)
        disp("Inizializzazione DB scenari...")
        path_DB_scenari_persistent = path_DB_scenari;
        data = load(path_DB_scenari_persistent); 
        DB_scenari = data.scenari; 
        episodi = 0;
        
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
    end
    
    % --- 2. Cambio di scenario ---
    try
        forced_idx = evalin('base', 'eval_scenario_idx');
        is_testing = ~isempty(forced_idx);
        scenario_corrente = forced_idx;
        disp(['Modalità Testing: Scenario forzato a ', num2str(scenario_corrente)]);
    catch
        is_testing = false;
    end
    
    if isempty(DB_scenari_eval)
        disp("Inizializzazione DB scenari di validation...")
        path_DB_scenari_eval = evalin('base', 'path_DB_scenari_eval');
        path_DB_scenari_eval_persistent = path_DB_scenari_eval;
        data = load(path_DB_scenari_eval_persistent); 
        DB_scenari_eval = data.scenari; 
    end
    
    if is_testing
        scenario_corrente = evalin('base', 'eval_scenario_idx');
        scenario = DB_scenari(scenario_corrente);
    elseif is_validation
        % disp("Cambio casuale di scenario di validation.")
        scenario_corrente = randi(length(DB_scenari_eval));
        scenario = DB_scenari_eval(scenario_corrente);
    else
        % disp("Modalità training: Cambio casuale di scenario")
        scenario_corrente = randi(length(DB_scenari));
        episodi = episodi + 1;
        scenario = DB_scenari(scenario_corrente);
    end
    
    % Assegnazioni Posizioni
    initial_pos = scenario.map.q_start; 
    init_vel = [0; 0; 0]; 
    init_euler = [0; 0; 0]; 
    bounds.x_min = squeeze(min(scenario.map.v(:,1,:))); bounds.x_max = squeeze(max(scenario.map.v(:,1,:)));
    bounds.y_min = squeeze(min(scenario.map.v(:,2,:))); bounds.y_max = squeeze(max(scenario.map.v(:,2,:)));
    bounds.z_min = squeeze(min(scenario.map.v(:,3,:))); bounds.z_max = squeeze(max(scenario.map.v(:,3,:)));
    
    % Workspace
    assignin('base', 'init_pos', initial_pos);
    assignin('base', 'init_vel', init_vel);
    assignin('base', 'init_euler', init_euler);
    assignin('base', 'sim_pos_des', scenario.sim_pos_des);
    assignin('base', 'sim_vel_des', scenario.sim_vel_des);
    assignin('base', 'sim_yaw_des', scenario.sim_yaw_des);
    assignin('base', 'pos_goal', scenario.map.q_goal);
    assignin('base', 'bounds', bounds);
    assignin('base', 'dyn_obs', scenario.dynamic_obstacles);
    
    max_distanza_goal = double(norm(initial_pos - scenario.map.q_goal));
    assignin('base', 'max_distanza_goal', max_distanza_goal);
    assignin('base', 'scenario_corrente', scenario_corrente);
end