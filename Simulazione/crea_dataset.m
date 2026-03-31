% =========================================================================
% ESTRAZIONE DATASET DEEN (2024 Dimensioni) - ULTRA-OTTIMIZZATA
% =========================================================================
clc;
disp('🚀 Inizio estrazione dataset a 10Hz (Versione Ultra-Veloce)...');

% -------------------------------------------------------------------------
% 1. PARAMETRI DI CAMPIONAMENTO E NORMALIZZAZIONE FISICA
% -------------------------------------------------------------------------
frequenza = 10; % Hz
dt = 1 / frequenza;
tempo_finale = out.UAVState.Position.Time(end); % Prende il tempo esatto dell'ultimo log
t_query = (0 : dt : tempo_finale - dt)'; 
num_campioni = length(t_query);

% COSTANTI DI NORMALIZZAZIONE (Limiti Fisici Assoluti del Drone)
MAX_VEL   = 30.0;    % Tetto massimo invalicabile (es. 108 km/h)
MAX_OMEGA = 2 * pi;  % Rotazione massima (es. 1 giro al secondo)
MAX_RPM   = 1000.0;  % RPM/PWM massimo dei motori

% -------------------------------------------------------------------------
% 2. INTERPOLAZIONE E NORMALIZZAZIONE VETTORIALE
% -------------------------------------------------------------------------
disp('📊 Interpolazione e normalizzazione segnali fisici...');

pos = interp1(out.UAVState.Position.Time, out.UAVState.Position.Data, t_query);

% Interpolazione e Clamping istantaneo tra -1 e 1
vel = max(min(interp1(out.UAVState.Velocity.Time, out.UAVState.Velocity.Data, t_query) / MAX_VEL, 1), -1);
omg = max(min(interp1(out.UAVState.Omega_b.Time, out.UAVState.Omega_b.Data, t_query) / MAX_OMEGA, 1), -1);
mot = (interp1(out.ActuatorCmds.Time, out.ActuatorCmds.Data, t_query) / MAX_RPM) * 2 - 1;

% Quaternioni: Interpolazione e Rinormalizzazione (per mantenere norma = 1)
quat = interp1(out.UAVState.Orientation.Time, out.UAVState.Orientation.Data, t_query);
quat = quat ./ sqrt(sum(quat.^2, 2)); 

% -------------------------------------------------------------------------
% 3. PRE-CALCOLO BOUNDING BOX OSTACOLI (Salva il 90% del tempo)
% -------------------------------------------------------------------------
disp('🏢 Pre-calcolo dell''ingombro dei 500 palazzi...');
bounds.x_min = squeeze(min(v(:,1,:))); bounds.x_max = squeeze(max(v(:,1,:)));
bounds.y_min = squeeze(min(v(:,2,:))); bounds.y_max = squeeze(max(v(:,2,:)));
bounds.z_min = squeeze(min(v(:,3,:))); bounds.z_max = squeeze(max(v(:,3,:)));

% -------------------------------------------------------------------------
% 4. LOOP DI ESTRAZIONE E ASSEMBLAGGIO
% -------------------------------------------------------------------------
disp('🏗️ Costruzione dei Voxel Locali e assemblaggio transizioni...');
Transitions = zeros(num_campioni - 1, 2024, 'single'); % Preallocazione

tic;
% Calcola il primissimo Voxel fuori dal loop (Shift-Trick)
vox_curr = estrai_cubo_voxel(pos(1,:), bounds);

for i = 1:(num_campioni - 1)
    
    % Dinamica attuale
    din_curr = [vel(i,:), omg(i,:), quat(i,:)];
    act_curr = mot(i,:);
    
    % Dinamica e Voxel futuri (calcoliamo SOLO il futuro)
    din_next = [vel(i+1,:), omg(i+1,:), quat(i+1,:)];
    vox_next = estrai_cubo_voxel(pos(i+1,:), bounds);
    
    % Assegnazione veloce della riga
    Transitions(i, :) = single([vox_curr, din_curr, act_curr, vox_next, din_next]);
    
    % SHIFT: Il voxel futuro diventa quello attuale per il prossimo ciclo
    % (Evita di calcolare due volte lo stesso identico cubo!)
    vox_curr = vox_next;
    
    if mod(i, 1000) == 0
        fprintf('   -> Elaborati %d/%d campioni...\n', i, num_campioni);
    end
end
tempo_estrazione = toc;
disp(['✅ Estrazione completata in soli ', num2str(tempo_estrazione), ' secondi.']);

% -------------------------------------------------------------------------
% 5. ESPORTAZIONE IN HDF5
% -------------------------------------------------------------------------
filename_h5 = 'dataset_singola_citta.h5';
if isfile(filename_h5)
    delete(filename_h5);
end

disp(['💾 Salvataggio nel formato HDF5 (', filename_h5, ')...']);
h5create(filename_h5, '/transitions', size(Transitions), 'Datatype', 'single');
h5write(filename_h5, '/transitions', single(Transitions));

disp('🎉 Dataset esportato! Pronto per il training PyTorch.');

% =========================================================================
% FUNZIONE AUSILIARIA: ESTREMIZZA LA VELOCITÀ DEI VOXEL
% =========================================================================
function voxel_flat = estrai_cubo_voxel(pos, b)
    res = 1.0; 
    r = 5; 
    margin = r * res;
    
    cubo = zeros(2*r, 2*r, 2*r, 'single');
    
    % Broad Phase Istantanea: usa le matrici pre-calcolate "b"
    vicini_idx = find(~(b.x_max < pos(1)-margin | b.x_min > pos(1)+margin | ...
                        b.y_max < pos(2)-margin | b.y_min > pos(2)+margin | ...
                        b.z_max < pos(3)-margin | b.z_min > pos(3)+margin));
                        
    if ~isempty(vicini_idx)
        % Matrici vettoriali (NDGRID calcolato solo se c'è un ostacolo vicino)
        vx_range = pos(1) - (r-0.5)*res : res : pos(1) + (r-0.5)*res;
        vy_range = pos(2) - (r-0.5)*res : res : pos(2) + (r-0.5)*res;
        vz_range = pos(3) - (r-0.5)*res : res : pos(3) + (r-0.5)*res;
        
        [VX, VY, VZ] = ndgrid(vx_range, vy_range, vz_range);
        
        for i = 1:length(vicini_idx)
            idx = vicini_idx(i);
            in_x = (VX >= b.x_min(idx)) & (VX <= b.x_max(idx));
            in_y = (VY >= b.y_min(idx)) & (VY <= b.y_max(idx));
            in_z = (VZ >= b.z_min(idx)) & (VZ <= b.z_max(idx));
            
            cubo(in_x & in_y & in_z) = 1;
        end
    end
    
    voxel_flat = cubo(:)';
end