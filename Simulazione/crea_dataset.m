% =========================================================================
% ESTRAZIONE DATASET DEEN (2024 Dimensioni) - CON QUATERNIONI DIRETTI
% Da eseguire con: simout, motori, tout, v, n_collision, quat_sim
% =========================================================================
disp('🚀 Inizio estrazione dataset a 10Hz (con Quaternioni diretti)...');

% -------------------------------------------------------------------------
% 1. GESTIONE TEMPI E CAMPIONAMENTO
% -------------------------------------------------------------------------
% Rimuoviamo i tempi duplicati (tipici dei solver a passo variabile)
[t_pos,  idx_pos]  = unique(simout.pos_vel.x.Time);
[t_att,  idx_att]  = unique(simout.attitude.rollspeed_p.Time);
[t_mot,  idx_mot]  = unique(tout);
[t_quat, idx_quat] = unique(quat_sim.Time);

frequenza = 10; % Hz
dt = 1 / frequenza;
t_query = (0 : dt : t_pos(end)-dt)';
num_campioni = length(t_query);

% Costanti di Normalizzazione (da adattare ai limiti fisici del tuo drone)
MAX_VEL   = 30.0;    % Tetto massimo di velocità lineare (m/s)
MAX_OMEGA = 2 * pi;  % Rotazione massima (rad/s)
MAX_RPM   = 1000.0;  % RPM o PWM massimo dei motori

% -------------------------------------------------------------------------
% 2. INTERPOLAZIONE E NORMALIZZAZIONE
% -------------------------------------------------------------------------
disp('📊 Interpolazione segnali fisici e normalizzazione...');

% Posizione (Non normalizzata, serve pura per la griglia Voxel)
pos_raw = [simout.pos_vel.x.Data(idx_pos), simout.pos_vel.y.Data(idx_pos), simout.pos_vel.z.Data(idx_pos)];
pos = interp1(t_pos, pos_raw, t_query);

% Velocità Lineare [-1, 1]
vel_raw = [simout.pos_vel.vx.Data(idx_pos), simout.pos_vel.vy.Data(idx_pos), simout.pos_vel.vz.Data(idx_pos)];
vel = max(min(interp1(t_pos, vel_raw, t_query) / MAX_VEL, 1), -1);

% Velocità Angolare [-1, 1]
omg_raw = [simout.attitude.rollspeed_p.Data(idx_att), simout.attitude.pitchspeed_q.Data(idx_att), simout.attitude.yawspeed_r.Data(idx_att)];
omg = max(min(interp1(t_att, omg_raw, t_query) / MAX_OMEGA, 1), -1);

% Motori [-1, 1]
mot_raw = motori(idx_mot, :);
mot = (interp1(t_mot, mot_raw, t_query) / MAX_RPM) * 2 - 1;

% Quaternioni Diretti (Interpolati e Rinormalizzati)
quat_raw = quat_sim.Data(idx_quat, :);
quat = interp1(t_quat, quat_raw, t_query);
quat = quat ./ sqrt(sum(quat.^2, 2)); % Fondamentale per ristabilire norma = 1

% -------------------------------------------------------------------------
% 3. PRE-CALCOLO OSTACOLI (Broad Phase Vettoriale)
% -------------------------------------------------------------------------
disp('🏢 Pre-calcolo dell''ingombro della città...');
bounds.x_min = squeeze(min(v(:,1,:))); bounds.x_max = squeeze(max(v(:,1,:)));
bounds.y_min = squeeze(min(v(:,2,:))); bounds.y_max = squeeze(max(v(:,2,:)));
bounds.z_min = squeeze(min(v(:,3,:))); bounds.z_max = squeeze(max(v(:,3,:)));

% -------------------------------------------------------------------------
% 4. COSTRUZIONE DATASET E VOXEL SHIFTING
% -------------------------------------------------------------------------
disp('🏗️ Estrazione Voxel e assemblaggio transizioni (2024 Dimensioni)...');
Transitions = zeros(num_campioni - 1, 2024, 'single');

tic;
% Estrai il primo Voxel fuori dal loop per innescare lo shift
vox_curr = estrai_cubo_voxel(pos(1,:), bounds);

for i = 1:(num_campioni - 1)
    
    % Stato Attuale t
    din_curr = [vel(i,:), omg(i,:), quat(i,:)]; % 3+3+4 = 10
    act_curr = mot(i,:);                        % 4
    
    % Stato Futuro t+1
    din_next = [vel(i+1,:), omg(i+1,:), quat(i+1,:)];
    vox_next = estrai_cubo_voxel(pos(i+1,:), bounds); % 1000
    
    % Scrittura riga
    Transitions(i, :) = single([vox_curr, din_curr, act_curr, vox_next, din_next]);
    
    % Shift: il futuro diventa il presente del prossimo ciclo (dimezza il tempo)
    vox_curr = vox_next;
    
    if mod(i, 1000) == 0
        fprintf('   -> Transizioni processate: %d / %d\n', i, num_campioni-1);
    end
end
disp(['✅ Array completato in ', num2str(toc), ' secondi.']);

% -------------------------------------------------------------------------
% 5. ESPORTAZIONE HDF5 PER IL TRAINING DEEN
% -------------------------------------------------------------------------
filename_h5 = 'dataset_singola_citta.h5';
if isfile(filename_h5)
    delete(filename_h5);
end

disp(['💾 Salvataggio in HDF5 (', filename_h5, ')...']);
h5create(filename_h5, '/transitions', size(Transitions), 'Datatype', 'single');
h5write(filename_h5, '/transitions', single(Transitions));
disp('🎉 FATTO! Dataset pronto per il modello in Python.');

% =========================================================================
% FUNZIONE AUSILIARIA PER I VOXEL "ON-THE-FLY"
% =========================================================================
function voxel_flat = estrai_cubo_voxel(pos, b)
    res = 5.0; r = 5; margin = r * res;
    cubo = zeros(2*r, 2*r, 2*r, 'single');
    
    vicini_idx = find(~(b.x_max < pos(1)-margin | b.x_min > pos(1)+margin | ...
                        b.y_max < pos(2)-margin | b.y_min > pos(2)+margin | ...
                        b.z_max < pos(3)-margin | b.z_min > pos(3)+margin));
                        
    if ~isempty(vicini_idx)
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