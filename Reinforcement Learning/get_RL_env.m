function env = get_RL_env(obsInfo, actInfo, path_DB_scenari, logging, logPath)
    
    if nargin < 3, path_DB_scenari = 'training_scenarios.mat'; end
    if nargin < 4, logging = false; logPath = fullfile(pwd, 'registro_morti.txt'); end
    assignin('base', 'logging', logging);
    logPath_padded = sprintf('%-250s', logPath);
    assignin('base', 'logPath_num', int8(logPath_padded));

    mdl = 'SAC_RL_env';
    agentBlk = [mdl, '/Inner Loop and Plant Model/High-FidelityModel/RL Agent'];
    
    % Creazione dell'ambiente Simulink
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    
    % Assegnazione all'ambiente della funzione di reset
    env.ResetFcn = @(in) localResetFcn(in, path_DB_scenari);

    disp('✅ Ambiente RL Simulink creato con successo');
end

function in = localResetFcn(in, path_DB_scenari)
    % Dichiarazione variabili persistenti
    persistent DB_scenari scenario_corrente tentativi_attuali max_tentativi
    persistent path_DB_scenari_persistent
    
    % Inizializzazione ad inizio training
    if isempty(DB_scenari) || ~strcmp(path_DB_scenari, path_DB_scenari_persistent)
        disp("Inizializzazione DB scenari...")
        % Carica il file .mat pre-calcolato una volta sola
        path_DB_scenari_persistent = path_DB_scenari;
        data = load(path_DB_scenari_persistent); 
        DB_scenari = data.scenari; 
        
        scenario_corrente = randi(length(DB_scenari)); % Primo scenario casuale
        tentativi_attuali = 0;
        max_tentativi = 1; % Quante volte si può riprovare la stessa mappa
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
    %x0 = scenario.map.q_start(1) + (rand() - 0.5) * 0.5; % +/- 25 cm
    %y0 = scenario.map.q_start(2) + (rand() - 0.5) * 0.5;
    %z0 = scenario.map.q_start(3);
    %initial_pos = [x0; y0; z0];
    %init_vel = [(rand()-0.5)*1.0; (rand()-0.5)*1.0; 0]; % +/- 0.5 m/s
    %init_euler = [(rand()-0.5)*0.2; (rand()-0.5)*0.2; 0]; % Roll e Pitch non nulli
    % Usa la posizione esatta
    initial_pos = scenario.map.q_start; 
    init_vel = [0; 0; 0]; % Parti da fermo
    init_euler = [0; 0; 0]; % Parti in hovering perfetto

    % Calcolo ingombro della città
    bounds.x_min = squeeze(min(scenario.map.v(:,1,:))); bounds.x_max = squeeze(max(scenario.map.v(:,1,:)));
    bounds.y_min = squeeze(min(scenario.map.v(:,2,:))); bounds.y_max = squeeze(max(scenario.map.v(:,2,:)));
    bounds.z_min = squeeze(min(scenario.map.v(:,3,:))); bounds.z_max = squeeze(max(scenario.map.v(:,3,:)));
    
    % Assegnazione variabili nel workspace
    assignin('base', 'init_pos', initial_pos);%rimosso ' vicino initial_pos
    assignin('base', 'init_vel', init_vel);
    assignin('base', 'init_euler', init_euler);
    assignin('base', 'sim_pos_des', scenario.sim_pos_des);
    assignin('base', 'sim_vel_des', scenario.sim_vel_des);
    assignin('base', 'sim_yaw_des', scenario.sim_yaw_des);
    assignin('base', 'pos_goal', scenario.map.q_goal);
    assignin('base', 'bounds', bounds);
    assignin('base', 'dyn_obs', scenario.dynamic_obstacles);

    assignin('base', 'scenario_corrente', scenario_corrente);
    
    disp(scenario_corrente)
    disp(path_DB_scenari_persistent)
    %disp(['✅ Punto spawn drone: [', num2str(initial_pos'), '], Goal a [', num2str(scenario.map.q_goal),']']);
    disp(['✅ Punto spawn drone: [', num2str(initial_pos(:)'), '], Goal a [', num2str(scenario.map.q_goal(:)'),']']);

end