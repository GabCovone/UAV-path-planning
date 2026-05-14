%% Script di Test Batch Ottimizzato per Agente SAC (test_agent.m)

lista_nomi = ["Agent3600"];

for i=1:length(lista_nomi)
    report_agent_fun(lista_nomi{i});
end

function report_agent_fun(nome_agente_file)
    clear eval_scenario_idx; clc; close all;
    
    %% 1. Configurazione Iniziale e Percorsi

    path_simulink = "SAC_RL_env/Inner Loop and Plant Model/High-FidelityModel/";
    load_system("SAC_RL_env");

    plantModelFi = 1;            
    useHeading = 1;              
    initialGainsMultiplier = 15;

    assignin('base', 'plantModelFi', plantModelFi);
    assignin('base', 'useHeading', useHeading);
    assignin('base', 'initialGainsMultiplier', initialGainsMultiplier);

    
    % Abilita il salvataggio su file della posizione
    block_path = strcat(path_simulink, "pos_agente To File");
    if get_param(block_path, 'Commented') == "on"
        set_param(block_path, 'Commented', 'off');
    end
    
    rng(2);
    Ts = 0.1;
    assignin('base', 'Ts', Ts);
    
    path_DB_scenari = 'validation_scenarios.mat';
    file_registro = fullfile(pwd, 'registro_morti.txt'); 
    
    % --- Parametrizzazione Nomi e Directory ---
    %nome_agente_file = 'Agent3300.mat'; 
    [~, nome_scenari_file, ~] = fileparts(path_DB_scenari); 
    
    % =========================================================================
    % SELEZIONE SCENARI DA VISUALIZZARE / TESTARE
    % Inserisci qui l'array con i numeri degli scenari che vuoi testare.
    % Esempio: [1, 4, 10]. Se vuoi eseguirli tutti, scrivi "tutti".
    %scenari_da_testare = [1, 2, 3]; 
    scenari_da_testare = "tutti";
    % =========================================================================
    
    % Creiamo la cartella dei report
    dir_report = fullfile('reports agenti', nome_agente_file);
    if ~exist(dir_report, 'dir')
        mkdir(dir_report);
    end
    
    %% 2. Inizializzazione Ambiente e Agente
    disp('Caricamento database scenari...');
    data_scenari = load(path_DB_scenari);
    num_totale_scenari_db = length(data_scenari.scenari);
    
    % Gestione della selezione scenari
    if isstring(scenari_da_testare) && scenari_da_testare == "tutti"
        scenari_da_testare = 1:num_totale_scenari_db;
    end
    num_scenari_selezionati = length(scenari_da_testare);
    
    [obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo();
    env = get_RL_env(obsInfo, actInfo, actLimit, path_DB_scenari, "validation_scenarios.mat", true, file_registro);
    
    % --- CARICAMENTO OTTIMIZZATO AGENTE ---
    agent_name = 'saved_agent'; 
    dati_salvati = load(strcat(nome_agente_file, ".mat"), agent_name);
    agente_valutazione = dati_salvati.(agent_name); 
    
    simOpts = rlSimulationOptions('MaxSteps', 5500, 'NumSimulations', 1);
    
    % Reset file di log
    if isfile(file_registro)
        delete(file_registro);
        disp('File registro resettato con successo.');
    end
    
    %% 3. Esecuzione Test Sequenziale
    fprintf('\n>>> AVVIO TEST SU %d SCENARI SELEZIONATI <<<\n', num_scenari_selezionati);
    
    for k = 1:num_scenari_selezionati
        % Estrae l'ID reale dello scenario da testare
        i = scenari_da_testare(k); 
        
        fprintf('\n--- Scenario %d (Test %d/%d) ---\n', i, k, num_scenari_selezionati);
        
        assignin('base', 'eval_scenario_idx', i);
        
        experience = sim(env, agente_valutazione, simOpts);
        
        reward_totale = sum(experience.Reward.Data);
        step_totali = length(experience.Reward.Data); 
        
        fprintf('Step: %d | Reward: %.2f\n', step_totali, reward_totale);
    
        % --- Recupero Telemetria e Creazione Figura ---
        if isfile('sim_pos_agente_1.mat')
            telemetria = load('sim_pos_agente_1.mat'); 
            
            if isfield(telemetria, 'sim_pos_agente') && ~isempty(telemetria.sim_pos_agente)
                dati_obj = getsamples(telemetria.sim_pos_agente, 2:telemetria.sim_pos_agente.Length);   
                dati_pos = dati_obj.Data;
                vettore_tempi = experience.Reward.Time;
                
                if size(dati_pos, 2) ~= 3
                    dati_pos = squeeze(dati_pos)'; 
                end
                
                titolo_finestra = sprintf('Traiettoria Scenario %d', i);
                fig_scenario = figure('Name', titolo_finestra, 'NumberTitle', 'off');
                
                graphic_func(path_DB_scenari, i, dati_pos, vettore_tempi); 
                
                nome_fig_scenario = sprintf('%s_%s_Scenario_%d.png', nome_agente_file, nome_scenari_file, i);
                percorso_fig_scenario = fullfile(dir_report, nome_fig_scenario);
                exportgraphics(fig_scenario, percorso_fig_scenario, 'Resolution', 300);
                
                % Se vuoi che le finestre si chiudano da sole dopo il salvataggio, togli il commento sotto:
                % close(fig_scenario); 
            end
        else
            warning('File telemetria non trovato per lo scenario %d.', i);
        end
    end
    
    evalin('base', 'clear eval_scenario_idx');
    disp('=== SIMULAZIONI COMPLETATE ===');
    
    %% 4. Analisi Risultati e Creazione Report
    fprintf('\nAnalisi del registro di sistema in corso...\n');
    pause(1); 
    
    try
        fid = fopen(file_registro, 'r');
        if fid == -1
            error('Impossibile aprire il file registro in: %s', file_registro);
        end
        testo_registro = fread(fid, '*char')'; 
        fclose(fid);
        
        % Parsing dei risultati
        num_successi   = count(testo_registro, "TRAGUARDO RAGGIUNTO!");
        num_collisioni = count(testo_registro, "Collisione ostacolo");
        num_schianti   = count(testo_registro, "Schianto a terra");
        num_deviazioni = count(testo_registro, "Deviazione eccessiva");
        num_timeout    = num_scenari_selezionati - (num_successi + num_collisioni + num_schianti + num_deviazioni);
        
        % --- Costruzione Nomi File Complessivi ---
        nome_file_base = sprintf('%s_%s', nome_agente_file, nome_scenari_file);
        percorso_file_txt = fullfile(dir_report, [nome_file_base, '_Riassunto.txt']);
        percorso_file_png = fullfile(dir_report, [nome_file_base, '_GraficoTorta.png']);
        
        % --- Costruzione Testo Report ---
        testo_report = sprintf('\n================ REPORT FINALE ================\n');
        testo_report = [testo_report, sprintf('Agente Valutato: %s\n', nome_agente_file)];
        testo_report = [testo_report, sprintf('Database Scenari: %s\n', path_DB_scenari)];
        testo_report = [testo_report, sprintf('Scenari Selezionati: %d su %d\n', num_scenari_selezionati, num_totale_scenari_db)];
        testo_report = [testo_report, sprintf('Successi:       %d (%.1f%%)\n', num_successi, (num_successi/num_scenari_selezionati)*100)];
        testo_report = [testo_report, sprintf('Collisioni:     %d (%.1f%%)\n', num_collisioni, (num_collisioni/num_scenari_selezionati)*100)];
        testo_report = [testo_report, sprintf('Schianti Terra: %d (%.1f%%)\n', num_schianti, (num_schianti/num_scenari_selezionati)*100)];
        testo_report = [testo_report, sprintf('Fuori Rotta:    %d (%.1f%%)\n', num_deviazioni, (num_deviazioni/num_scenari_selezionati)*100)];
        testo_report = [testo_report, sprintf('Timeout:        %d (%.1f%%)\n', num_timeout, (num_timeout/num_scenari_selezionati)*100)];
        testo_report = [testo_report, sprintf('===============================================\n')];
        
        fprintf('%s', testo_report);
        
        fid_txt = fopen(percorso_file_txt, 'w');
        if fid_txt ~= -1
            fprintf(fid_txt, '%s', testo_report);
            fclose(fid_txt);
            fprintf('✅ File di testo salvato in:\n   -> %s\n', percorso_file_txt);
        end
        
        % --- Generazione Grafico a Torta ---
        dati_torta = [num_successi, num_collisioni, num_schianti, num_deviazioni, num_timeout];
        etichette = {'Successo', 'Collisione', 'Schianto a Terra', 'Fuori Rotta', 'Timeout'};
        idx_validi = dati_torta > 0; 
        
        if any(idx_validi)
            fig_torta = figure('Name', 'Risultati Valutazione', 'NumberTitle', 'off', 'Visible', 'off');
            pie(dati_torta(idx_validi), etichette(idx_validi));
            
            titolo_grafico = sprintf('Risultati: %s\nsu %d Scenari Selezionati', nome_agente_file, num_scenari_selezionati);
            title(titolo_grafico, 'Interpreter', 'none');
            
            colormap(gca, [0.4660 0.6740 0.1880; 0.8500 0.3250 0.0980; 0.6350 0.0780 0.1840; 0.9290 0.6940 0.1250; 0.5 0.5 0.5]);
            
            exportgraphics(fig_torta, percorso_file_png, 'Resolution', 300);
            fprintf('✅ Grafico a torta esportato in:\n   -> %s\n', percorso_file_png);
            
            close(fig_torta); 
        else
            disp('Nessun dato valido nel registro per generare il grafico a torta.');
        end
        
    catch ME
        fprintf('❌ Errore durante l''analisi del file di log:\n%s\n', ME.message);
    end
end