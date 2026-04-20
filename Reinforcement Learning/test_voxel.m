%% Script: ispettore_voxel.m
disp('Avvio ispettore Voxel in finestra indipendente...');

disp('Scansione di tutto il volo alla ricerca di ostacoli...');

% Estraiamo tutti i dati del sensore
data_voxels = experience.Observation.Voxels_Observations.Data;

% Si estrae l'istante di ciascuno step
tempi_step = experience.Observation.Voxels_Observations.Time;

% Sommiamo i valori per ogni step per vedere quanti voxel erano a 1
if ndims(data_voxels) == 4
    % Caso [10 x 10 x 10 x N]
    somma_voxel_per_step = squeeze(sum(data_voxels, [1 2 3]));
elseif ndims(data_voxels) == 3
    % Caso [1000 x 1 x N] o [10 x 100 x N]
    somma_voxel_per_step = squeeze(sum(sum(data_voxels, 1), 2));
else
    somma_voxel_per_step = sum(data_voxels, 1)';
end

% Troviamo in quali step il sensore ha percepito qualcosa
step_con_ostacoli = find(somma_voxel_per_step > 0);

if isempty(step_con_ostacoli)
    disp('❌ ALLARME: Il sensore è rimasto a ZERO per tutto il volo!');
else
    disp('✅ Il sensore FUNZIONA! Ha visto ostacoli nei seguenti step:');
    disp(['Primo avvistamento allo step: ', num2str(step_con_ostacoli(1))]);
    disp(['Ultimo avvistamento allo step: ', num2str(step_con_ostacoli(end))]);
    disp(['Voxel massimi visti in un singolo step: ', num2str(max(somma_voxel_per_step))]);
end


try
    % 1. Scegli quale momento del volo vuoi guardare
    % (In questo caso l'ultimo step prima della fine)
    %step_da_analizzare = length(experience.Reward.Data);

    % 0.005

    tempo_da_analizzare = 0.1;

    tempo_per_step = round(tempo_da_analizzare,1)

    step_da_analizzare = find(tempi_step == tempo_per_step)

    % 2. Estrai la posizione del drone a quello step
    idx = find(sim_pos_agente.Time == tempo_da_analizzare);
    pos_corrente = sim_pos_agente.Data(idx,:);
    
    % 3. Estrazione intelligente dei Voxel
    data_voxels = experience.Observation.Voxels_Observations.Data;
    
    if ndims(data_voxels) == 4
        % Se Simulink lo ha salvato come 4D [10 x 10 x 10 x N]
        voxel_corrente = data_voxels(:, :, :, step_da_analizzare);
    elseif ndims(data_voxels) == 3
        % Se Simulink lo ha salvato come [1000 x 1 x N] o [10 x 100 x N]
        voxel_corrente = data_voxels(:, :, step_da_analizzare);
    else
        % Fallback generico
        voxel_corrente = data_voxels(:, step_da_analizzare);
    end
    
    % 4. Apri la finestra indipendente
    visualizza_voxel(voxel_corrente, pos_corrente, 5.0, 5);
    
catch ME
    disp('❌ Errore durante l''estrazione dei dati:');
    disp(ME.message);
end