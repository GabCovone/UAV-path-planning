function env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, logging, logPath)
    
    if nargin < 4, path_DB_scenari = 'training_scenarios.mat'; end
    if nargin < 5, logging = false; logPath = fullfile(pwd, 'registro_morti.txt'); end
    assignin('base', 'logging', logging);
    logPath_padded = sprintf('%-250s', logPath);
    assignin('base', 'logPath_num', int8(logPath_padded));

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

    assignin('base', 'tolleranza_goal', 2);
    
    % Creazione dell'ambiente Simulink
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    
    % Assegnazione all'ambiente della funzione di reset
    env.ResetFcn = @(in) localResetFcn(in, path_DB_scenari);

    disp('✅ Ambiente RL Simulink creato con successo');
end

function in = localResetFcn(in, path_DB_scenari)
    % Dichiarazione variabili persistenti
    persistent DB_scenari scenario_corrente episodi
    persistent path_DB_scenari_persistent
    
    % --- 1. Inizializzazione ad inizio training o se cambia il file del DB ---
    if isempty(DB_scenari) || ~strcmp(path_DB_scenari, path_DB_scenari_persistent)
        disp("Inizializzazione DB scenari...")
        % Carica il file .mat pre-calcolato una volta sola
        path_DB_scenari_persistent = path_DB_scenari;
        data = load(path_DB_scenari_persistent); 
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
    end
    
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

    % Aggiornamento contatore degli episodi
    episodi = episodi + 1;
    
    % Si estraggono i dati dello scenario da usare in questo episodio
    scenario = DB_scenari(scenario_corrente);
    
    % --- Resto della funzione inalterato ---
    % Usa la posizione esatta
    initial_pos = scenario.map.q_start; % è 1 x 3, a differenza di velocità e orientamento
    init_vel = [0; 0; 0]; % Parti da fermo
    init_euler = [0; 0; 0]; % Parti in hovering perfetto

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
    assignin('base', 'pos_goal', scenario.map.q_goal); % Si traspone in modo che sia 3 x 1
    assignin('base', 'bounds', bounds);
    assignin('base', 'dyn_obs', scenario.dynamic_obstacles);

    % IMPORTANTISSIMO: questa variabile ora sarà 'i' durante il loop di test!
    assignin('base', 'scenario_corrente', scenario_corrente);
    
    disp(['✅ Punto spawn drone: [', num2str(initial_pos(:)'), '], Goal a [', num2str(scenario.map.q_goal(:)'),']']);

end