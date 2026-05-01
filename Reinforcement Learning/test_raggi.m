%% Script: ispettore_voxel.m
disp('Avvio ispettore Raggi in finestra indipendente...');

%disp('Scansione di tutto il volo alla ricerca di ostacoli...');


sim_pos_agente = load('sim_pos_agente_1.mat')
sim_pos_agente = sim_pos_agente.sim_pos_agente;

sim_rays = load('sim_rays_1.mat');
sim_rays = sim_rays.sim_rays;

data_raggi = sim_rays.Data;

tempi_step = sim_rays.Time;

% Sommiamo i valori per ogni step per vedere quanti voxel erano a 1
if ndims(data_raggi) == 3
    % Caso [256 x 1 x N] o [1 x 256 x N]
    somma_raggi_per_step = squeeze(sum(sum(data_raggi, 1), 2));
end
step_con_ostacoli = find(somma_raggi_per_step ~= 256);
if isempty(step_con_ostacoli)
    disp('❌ ALLARME: Il sensore è rimasto a ZERO per tutto il volo!');
else
    disp('✅ Il sensore FUNZIONA! Ha visto ostacoli nei seguenti step:');
    disp(['Primo avvistamento allo step: ', num2str(step_con_ostacoli(1))]);
    disp(['Ultimo avvistamento allo step: ', num2str(step_con_ostacoli(end))]);
end



% 1. Scegli quale momento del volo vuoi guardare
% (In questo caso l'ultimo step prima della fine)
%step_da_analizzare = length(experience.Reward.Data);

% 0.005

tempo_da_analizzare = 5; %39.1;

tempo_per_step = round(tempo_da_analizzare,1)

% step_da_analizzare = find(tempi_step == 39.8)

step_da_analizzare = tempo_per_step * 10 + 1;

% 2. Estrai la posizione del drone a quello step
idx = find(sim_pos_agente.Time == tempo_per_step);
pos_corrente = sim_pos_agente.Data(idx,:)

% 3. Estrazione intelligente dei Raggi

raggi_corrente = data_raggi(:, :, step_da_analizzare)

length(raggi_corrente)

% if ndims(data_raggi) == 4
%     % Se Simulink lo ha salvato come 4D [10 x 10 x 10 x N]
%     voxel_corrente = data_raggi(:, :, :, step_da_analizzare);
% elseif ndims(data_raggi) == 3
%     % Se Simulink lo ha salvato come [1000 x 1 x N] o [10 x 100 x N]
%     voxel_corrente = data_raggi(:, :, step_da_analizzare);
% else
%     % Fallback generico
%     voxel_corrente = data_raggi(:, step_da_analizzare);
% end

% 4. Apri la finestra indipendente

idx_scenario = 1;
path_DB_scenari = 'training_scenarios.mat';

visualizza_raggi(pos_corrente, raggi_corrente);

%graphic_func(path_DB_scenari, idx_scenario, sim_pos_agente.Data, tempo_per_step);

