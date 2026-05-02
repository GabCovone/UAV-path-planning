%% 4. Analisi Risultati (Parsing del Log)

file_name = 'registro_morti_v17_lv4_v2.txt';

file_registro = fullfile(pwd, strcat('\versioni_agenti\', file_name)); 

fprintf('\nAnalisi del registro di sistema in corso...\n');
pause(1); % Assicura che Simulink abbia rilasciato l'handle del file


% Esegui il parsing
episodes = parseRLTrainingLog(file_registro);

% Visualizza i dati come tabella
T = struct2table(episodes);

% Accedi ai dati singolarmente
num_episodes = length(episodes);
episodi = 1:num_episodes;
steps = [episodes.step];           % Vettore di tutti gli step
rewards = [episodes.reward];       % Vettore di tutti i reward totali
distances = [episodes.distanza_goal];  % Vettore delle distanze dal goal
penalties_actions = [episodes.azioni];       % Vettore della penalità su azioni
penalties_obstacles = [episodes.ostacoli];   % Vettore della penalità su ostacoli
penalites_position = [episodes.posizione];  % Vettore della penalità su deviazione posizione
rewards_progress = [episodes.progresso];   % Vettore del reward su progresso

% Grafico dell'evoluzione del reward per episodio
figure;
plot(rewards, 'o-', 'LineWidth', 1.5);
xlabel('Episodio');
ylabel('Reward Totale');
title('Evoluzione del Reward');
grid on;

% ===== FIGURA 1: COMPONENTI DEL REWARD SOVRAPPOSTE =====
figure('Position', [100 100 1200 600], 'Name', 'Componenti del Reward');

hold on;
plot(episodi, penalties_actions, 'o-', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Azioni');
plot(episodi, penalties_obstacles, 's-', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Ostacoli');
plot(episodi, penalites_position, '^-', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Posizione');
plot(episodi, rewards_progress, 'd-', 'LineWidth', 2, 'MarkerSize', 5, 'DisplayName', 'Progresso');

% Linea del reward totale in evidenza (più spessa e trasparente)
%plot(episodi, reward, 'k-', 'LineWidth', 3, 'Alpha', 0.3, 'DisplayName', 'Reward Totale');

xlabel('Episodio', 'FontSize', 12);
ylabel('Valore del Componente', 'FontSize', 12);
title('Componenti del Reward al variare degli Episodi', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
grid minor;
hold off;

% ===== FIGURA 2: REWARD TOTALE, DISTANZA DAL GOAL =====
figure('Position', [100 750 1200 600], 'Name', 'Reward e Goal Distance');

tiledlayout(2, 1, 'TileSpacing', 'compact');

% Subplot 1: Reward totale
nexttile;
bar(episodi, rewards, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'black', 'LineWidth', 1.5);
xlabel('Episodio', 'FontSize', 11);
ylabel('Reward Totale', 'FontSize', 11);
title('Reward Totale per Episodio', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
grid minor;

% Subplot 2: Distanza dal goal
nexttile;
plot(episodi, distances, 'o-', 'LineWidth', 2.5, 'MarkerSize', 6, 'Color', [0.8 0.2 0.2]);
xlabel('Episodio', 'FontSize', 11);
ylabel('Distanza dal Goal (m)', 'FontSize', 11);
title('Distanza dal Goal al variare degli Episodi', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
grid minor;

% ===== FIGURA 3: STEP PER EPISODIO =====
figure('Position', [100 1400 1200 500], 'Name', 'Steps per Episodio');

bar(episodi, steps, 'FaceColor', [0.2 0.8 0.2], 'EdgeColor', 'black', 'LineWidth', 1.5);
xlabel('Episodio', 'FontSize', 12);
ylabel('Numero di Step', 'FontSize', 12);
title('Step Eseguiti per Episodio', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
grid minor;


function episodes = parseRLTrainingLog(filename)
    % Leggi il file riga per riga
    lines = readlines(filename);
    
    % Inizializza struttura per gli episodi
    episodes = struct('step', {}, 'reward', {}, 'distanza_goal', {}, ...
                      'azioni', {}, 'ostacoli', {}, 'posizione', {}, 'progresso', {});
    
    episode_idx = 0;
    i = 1;
    
    while i <= length(lines)
        line = lines(i);
        
        % Individua righe che iniziano con "Step:"
        if contains(line, 'Step:') && contains(line, 'Rwd:')
            episode_idx = episode_idx + 1;
            
            % PARSING PRIMA RIGA
            % Step: 190 | Rwd: -17.81 | Distanza goal: 911.73 | Motivo: ...
            pattern1 = 'Step:\s*(\d+)\s*\|\s*Rwd:\s*([-\d.]+)\s*\|\s*Distanza goal:\s*([-\d.]+)';
            tokens = regexp(line, pattern1, 'tokens');
            
            if ~isempty(tokens)
                episodes(episode_idx).step = str2double(tokens{1}{1});
                episodes(episode_idx).reward = str2double(tokens{1}{2});
                episodes(episode_idx).distanza_goal = str2double(tokens{1}{3});
            end
            
            % PARSING SECONDA RIGA (dati dettagliati)
            if i + 1 <= length(lines)
                line2 = lines(i + 1);
                % azioni: -445.240546 | ostacoli: -390.670702 | posizione: -41.466730 | ... | progresso: 1402.966309
                pattern2 = 'azioni:\s*([-\d.]+)\s*\|\s*ostacoli:\s*([-\d.]+)\s*\|\s*posizione:\s*([-\d.]+).*progresso:\s*([-\d.]+)';
                tokens2 = regexp(line2, pattern2, 'tokens');
                
                if ~isempty(tokens2)
                    episodes(episode_idx).azioni = str2double(tokens2{1}{1});
                    episodes(episode_idx).ostacoli = str2double(tokens2{1}{2});
                    episodes(episode_idx).posizione = str2double(tokens2{1}{3});
                    episodes(episode_idx).progresso = str2double(tokens2{1}{4});
                end
                
                i = i + 2; % Salta la riga appena letta
            else
                i = i + 1;
            end
        else
            i = i + 1;
        end
    end
end