%% Script di Verifica Visiva dei Dataset Generati
% Questo script carica il primo scenario da ogni file .mat presente nella
% cartella corrente e lo visualizza in un grafico 3D per un controllo rapido.
%
% ISTRUZIONI:
% 1. Posizionare questo script nella cartella 'Datasets'.
% 2. Eseguirlo da MATLAB.

clear; clc; close all;

disp('--- Avvio Script di Verifica Visiva dei Dataset ---');

% 1. Trova tutti i file .mat nella cartella corrente
file_lista = dir('*_L2.mat');

if isempty(file_lista)
    disp('Nessun file .mat trovato in questa cartella. Assicurati di essere nella cartella "Datasets".');
    return;
end

fprintf('Trovati %d file .mat. Inizio la visualizzazione...\n\n', length(file_lista));

% 2. Cicla su ogni file trovato
for i = 1:length(file_lista)
    nome_file = file_lista(i).name;
    fprintf('--- Caricamento e visualizzazione di: %s ---\n', nome_file);
    
    try
        % Carica il contenuto del file .mat
        data = load(nome_file);
        
        if ~isfield(data, 'scenari') || isempty(data.scenari)
            fprintf('   -> ⚠️ File vuoto o non contiene la variabile "scenari". Salto.\n\n');
            continue;
        end
        
        % Prendi il primo scenario come campione
        scenario_campione = data.scenari(1);
        
        % Crea una nuova figura per questo scenario
        figure('Name', nome_file, 'NumberTitle', 'off');
        hold on;
        
        mappa = scenario_campione.map;
        traiettoria = scenario_campione.sim_pos_des;
        
        % Plotta gli edifici (se presenti)
        if isfield(mappa, 'v') && ~isempty(mappa.v)
            plot_buildings(mappa.v);
        end
        
        % Plotta gli ostacoli dinamici (se presenti)
        if isfield(scenario_campione, 'dynamic_obstacles') && ~isempty(scenario_campione.dynamic_obstacles)
            plot_dynamic_obstacles(scenario_campione.dynamic_obstacles);
        end
        
        % Plotta la traiettoria di riferimento, lo start e il goal
        plot3(traiettoria.Data(:,1), traiettoria.Data(:,2), traiettoria.Data(:,3), 'b-', 'LineWidth', 2);
        plot3(mappa.q_start(1), mappa.q_start(2), mappa.q_start(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
        plot3(mappa.q_goal(1), mappa.q_goal(2), mappa.q_goal(3), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
        
        title(['Scenario Campione da: ', nome_file], 'Interpreter', 'none');
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        axis equal; grid on; view(30, 30); hold off;
        
        fprintf('   -> ✅ Grafico generato con successo.\n\n');
        
    catch ME
        fprintf('   -> ❌ ERRORE durante l''elaborazione del file %s: %s\n\n', nome_file, ME.message);
    end
end

disp('--- Verifica completata. ---');

%% Funzione ausiliaria per il plotting degli edifici
function plot_buildings(v)
    num_buildings = size(v, 3);
    faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    for k = 1:num_buildings
        patch('Faces', faces, 'Vertices', v(:,:,k), 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
end

%% Funzione ausiliaria per il plotting degli ostacoli dinamici
function plot_dynamic_obstacles(dyn_obs)
    % Itera su tutti gli ostacoli dinamici e li plotta
    for j = 1:length(dyn_obs)
        obs = dyn_obs(j);
        % Non plotta gli ostacoli "finti" usati per il padding (raggio nullo)
        if obs.radius > 0 
            % Plotta una sfera per rappresentare l'ostacolo e il suo raggio
            [x,y,z] = sphere;
            x_sphere = x * obs.radius + obs.p0(1);
            y_sphere = y * obs.radius + obs.p0(2);
            z_sphere = z * obs.radius + obs.p0(3);
            surf(x_sphere, y_sphere, z_sphere, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.4);
        end
    end
end