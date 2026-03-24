clearvars
close all

disp('Inizializzazione simulatore urbano (8000x8000x8000)...');
tic

% =========================================================================
% 1. PARAMETRI DI BASE E PREALLOCAZIONE
% =========================================================================
x_max = 8000; y_max = 8000; z_max = 8000;
frame_range = [x_max, y_max, z_max];
numNodes = 30000; % Elevato numero di nodi per mappare la città gigante

% Partenza e Arrivo a 1 metro da terra
q_start.coord = [0 0 1]; 
q_start.cost = 0;
q_start.parent = 0;

q_goal.coord = [6900 7360 1]; 
q_goal.cost = 0;

% Preallocazione della memoria (evita il collasso della RAM)
nodes = repmat(q_start, 1, numNodes + 500); 
nodes(1) = q_start;
node_count = 1;

% =========================================================================
% 2. GENERAZIONE PROCEDURALE DELLA CITTÀ
% =========================================================================
n_collision = 500;
rng(42); % Fissa il seme per avere la stessa città ad ogni avvio (Dataset coerente)

% Genera la città (vettorizzata e istantanea)
[v, creat_center, lengthxyz] = generateCity(n_collision, x_max);

% Calcolo dei centri esatti e dei raggi d'ingombro per la Repulsione Fisica (APF)
exp_center.coord = [0 0 0];
v_center = repmat(exp_center, 1, n_collision);
v_r = zeros(1, n_collision);

for num = 1:n_collision
    v_center(num).coord = [mean([min(v(:,1,num)), max(v(:,1,num))]), ...
                           mean([min(v(:,2,num)), max(v(:,2,num))]), ...
                           mean([min(v(:,3,num)), max(v(:,3,num))])];
    v_r(num) = dist_3d(v(1,:,num), v_center(num).coord);
end

% =========================================================================
% 3. CICLO PRINCIPALE: APF-RRT* OTTIMIZZATO
% =========================================================================
disp('Ricerca del percorso ottimale (RRT*) in corso...');
for i = 1:numNodes
    r_create = 800; % Raggio di esplorazione ampio
    q_rand = creatPoint(frame_range, nodes(node_count), r_create, v, n_collision, q_goal.coord);
    
    % Vettorizzazione: Ricerca istantanea del nodo più vicino
    coords_mat = reshape([nodes(1:node_count).coord], 3, node_count)'; 
    dists_to_rand = sqrt(sum((coords_mat - q_rand).^2, 2));
    [~, idx] = min(dists_to_rand);
    q_near = nodes(idx);
    
    % APF (Attrazione verso il goal + Repulsione dagli ostacoli)
    pho = 50; % Lunghezza del passo
    k_att = 1.5; k_rep = 1.5;
    q_new_coord = steer3d(q_rand, q_near.coord, q_goal.coord, pho, k_att, frame_range, k_rep, v_center, v_r, n_collision);
    
    % Controllo Collisioni
    if collisionChecking(q_new_coord, q_near.coord, v, n_collision)
        q_new.coord = q_new_coord;
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
        q_new.parent = idx;
        
        % Vettorizzazione: Rewiring (ottimizzazione dei rami dell'albero)
        r = 150; 
        dists_to_new = sqrt(sum((coords_mat - q_new.coord).^2, 2));
        idx_near = find(dists_to_new <= r)';
        
        q_min = q_near;
        C_min = q_new.cost;
        
        for j = idx_near
            if (nodes(j).cost + dist_3d(nodes(j).coord, q_new.coord) < C_min) && collisionChecking(q_new.coord, nodes(j).coord, v, n_collision)
                q_min = nodes(j);
                C_min = nodes(j).cost + dist_3d(nodes(j).coord, q_new.coord);
                q_new.parent = j;
                q_new.cost = C_min;
            end
        end
        
        % Salvataggio del nuovo nodo nell'albero
        node_count = node_count + 1;
        nodes(node_count) = q_new;
        
        % Controllo se siamo arrivati a destinazione (Tolleranza di un passo)
        if dist_3d(q_new.coord, q_goal.coord) < 55
            disp('🎯 Traguardo raggiunto a 1 metro da terra!');
            break;
        end
    end
end

disp(['Nodi generati: ', num2str(node_count)]);
disp(['Costo percorso grezzo: ', num2str(nodes(node_count).cost)]);

% =========================================================================
% 4. ESTRAZIONE, PRUNING E GRAFICA
% =========================================================================
% 4.1 Trova il nodo più vicino al traguardo e collega la meta
D = zeros(1, node_count);
for j = 1:node_count
    D(j) = dist_3d(nodes(j).coord, q_goal.coord);
end

[~, idx] = min(D);
q_goal.parent = idx;
node_count = node_count + 1;
nodes(node_count) = q_goal;

% Risaliamo l'albero per ottenere gli indici (Ottimizzato con Preallocazione)
wp_indices_temp = zeros(1, node_count); 
idx_count = node_count; 
curr_node = node_count;

while curr_node ~= 0
    wp_indices_temp(idx_count) = curr_node; 
    idx_count = idx_count - 1;
    curr_node = nodes(curr_node).parent;
end

% Tagliamo via tutti gli zeri rimasti vuoti all'inizio
wp_indices = wp_indices_temp(idx_count + 1 : end);

% Matrice N x 3 dei Waypoint GREZZI (zig-zag dell'RRT*)
waypoints_raw = reshape([nodes(wp_indices).coord], 3, length(wp_indices))';

% -------------------------------------------------------------------------
% 4.2 L'ALGORITMO DI PRUNING (Line-of-Sight) - OTTIMIZZATO
% -------------------------------------------------------------------------
disp('Esecuzione Pruning (Line-of-Sight)...');

% PREALLOCAZIONE per la massima velocità
max_wp = size(waypoints_raw, 1);
waypoints_pruned_temp = zeros(max_wp, 3); 

% Inseriamo il primo punto (Start)
waypoints_pruned_temp(1, :) = waypoints_raw(1, :); 
pruned_count = 1; 
curr_idx = 1;

while curr_idx < max_wp
    next_idx = max_wp; 
    
    while next_idx > curr_idx + 1
        if collisionChecking(waypoints_raw(next_idx, :), waypoints_raw(curr_idx, :), v, n_collision)
            break; 
        end
        next_idx = next_idx - 1; 
    end
    
    pruned_count = pruned_count + 1;
    waypoints_pruned_temp(pruned_count, :) = waypoints_raw(next_idx, :);
    curr_idx = next_idx;
end

% TAGLIO FINALE
waypoints_pruned = waypoints_pruned_temp(1:pruned_count, :);

disp(['Waypoint grezzi originali: ', num2str(max_wp)]);
disp(['Waypoint essenziali (Post-Pruning): ', num2str(pruned_count)]);
disp(['Tempo di calcolo totale: ', num2str(toc), ' secondi']);

% -------------------------------------------------------------------------
% 4.3 GRAFICA DELLA CITTÀ E DEI PERCORSI
% -------------------------------------------------------------------------
figure(1); hold on;
title('APF-RRT* + Pruning in Mappa Urbana 3D');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% 1. Disegno della città (Palazzi 3D traslucidi)
disp('Disegno della città 3D in corso...');
for k = 1:n_collision
    V_b = v(:,:,k); 
    f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
end

% 2. Disegna il percorso GREZZO (Rosso, tratteggiato)
plot3(waypoints_raw(:,1), waypoints_raw(:,2), waypoints_raw(:,3), 'Color', [1 0.3 0.3], 'LineWidth', 1.5, 'LineStyle', '--');

% 3. Disegna il percorso PRUNED (Verde fluo, con i marker)
plot3(waypoints_pruned(:,1), waypoints_pruned(:,2), waypoints_pruned(:,3), 'Color', 'g', 'LineWidth', 4);
plot3(waypoints_pruned(:,1), waypoints_pruned(:,2), waypoints_pruned(:,3), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y');

% 4. Marker di Partenza e Arrivo
plot3(q_start.coord(1), q_start.coord(2), q_start.coord(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(q_goal.coord(1), q_goal.coord(2), q_goal.coord(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');

view(30,30); axis equal; grid on; hold off;

% -------------------------------------------------------------------------
% 5. SALVATAGGIO DEI FILE PER IL DATASET
% -------------------------------------------------------------------------
save('waypoints_estratti.mat', 'waypoints_pruned');
disp('Waypoints salvati con successo in "waypoints_estratti.mat".');

save('mappa_urbana.mat', 'v', 'n_collision', 'x_max', 'y_max', 'z_max');
disp('Mappa 3D salvata con successo in "mappa_urbana.mat".');

% =========================================================================
% FUNZIONI LOCALI (MOTORE MATEMATICO, FISICO E URBANO) - SUPER OTTIMIZZATO
% =========================================================================

function [v, creat_center, lengthxyz] = generateCity(n_collision, max_xy)
    creat_center = zeros(n_collision, 3);
    lengthxyz = zeros(n_collision, 3);
    v = zeros(8, 3, n_collision);
    
    min_pos = 100; max_pos = max_xy - 400; 
    min_len = 100; max_len = 300; 
    margine = 30; % Distanza minima tra gli edifici
    
    i = 1;
    while i <= n_collision
        cand_center = [min_pos + (max_pos - min_pos) * rand(1, 2), 0]; 
        cand_len = [min_len + (max_len - min_len) * rand(1, 2), 0];
        
        prob = rand();
        if prob < 0.60
            cand_len(3) = 50 + 100 * rand();       % 60% Palazzi Bassi (50-150m)
        elseif prob < 0.90
            cand_len(3) = 150 + 250 * rand();      % 30% Palazzi Medi (150-400m)
        else
            cand_len(3) = 400 + 400 * rand();      % 10% Grattacieli (400-800m)
        end
        
        cand_min = cand_center(1:2);
        cand_max = cand_center(1:2) + cand_len(1:2);
        
        if i > 1
            esist_min = creat_center(1:i-1, 1:2);
            esist_max = creat_center(1:i-1, 1:2) + lengthxyz(1:i-1, 1:2);
            
            overlap_x = (cand_min(1) < esist_max(:,1) + margine) & (cand_max(1) + margine > esist_min(:,1));
            overlap_y = (cand_min(2) < esist_max(:,2) + margine) & (cand_max(2) + margine > esist_min(:,2));
            sovrapposto = any(overlap_x & overlap_y);
        else
            sovrapposto = false;
        end
        
        if ~sovrapposto
            creat_center(i, :) = cand_center;
            lengthxyz(i, :) = cand_len;
            i = i + 1;
        end
    end
    
    for k = 1:n_collision
        point1 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2)+lengthxyz(k,2), lengthxyz(k,3)];
        point2 = [creat_center(k,1),                creat_center(k,2)+lengthxyz(k,2), lengthxyz(k,3)];
        point3 = [creat_center(k,1),                creat_center(k,2),                lengthxyz(k,3)];
        point4 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2),                lengthxyz(k,3)];
        point5 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2)+lengthxyz(k,2), 0];
        point6 = [creat_center(k,1),                creat_center(k,2)+lengthxyz(k,2), 0];
        point7 = [creat_center(k,1),                creat_center(k,2),                0];
        point8 = [creat_center(k,1)+lengthxyz(k,1), creat_center(k,2),                0];
        
        v(:,:,k) = [point1; point2; point3; point4; point5; point6; point7; point8];
    end
end

function qnewx = Repulsion(qnew, k_rep, v_center, v_r, n_collision)
    centers_mat = reshape([v_center(1:n_collision).coord], 3, n_collision)';
    diff_vec = qnew - centers_mat; 
    dists = sqrt(sum(diff_vec.^2, 2)); 
    
    idx_close = dists < (1.2 * v_r');
    
    if any(idx_close)
        close_diffs = diff_vec(idx_close, :);
        close_dists = dists(idx_close);
        forces = k_rep * (close_diffs ./ close_dists);
        F_rep = sum(forces, 1); 
    else
        F_rep = [0 0 0];
    end
    
    qnewx = qnew + F_rep;
end

function qnew = steer3d(q_rand, q_near, q_goal, pho, k_att, frame_range, k_rep, v_center, v_r, n_collision)
   dir_rand = (q_rand - q_near) / (dist_3d(q_rand, q_near) + 1e-6);
   dir_goal = (q_goal - q_near) / (dist_3d(q_goal, q_near) + 1e-6);
   qnew = q_near + pho * (dir_rand + k_att * dir_goal);
   
   qnew = Repulsion(qnew, k_rep, v_center, v_r, n_collision);
   if ~InFrame(frame_range, qnew)
       qnew = q_rand;
   end
end

function d = dist_3d(q1, q2)
    d = norm(q1 - q2); 
end

% -------------------------------------------------------------------------
% VETTORIZZAZIONE 2: CONTROLLO OSTACOLI CON MARGINE DI SICUREZZA
% -------------------------------------------------------------------------
function feasible = in_obstacle(q, v, n)
    % MARGINE DI SICUREZZA: Gonfiamo gli ostacoli per non tagliare gli angoli!
    margin = 10.0; % 10 metri di rispetto dal muro
    
    x_min = squeeze(min(v(:,1,:))) - margin; x_max = squeeze(max(v(:,1,:))) + margin;
    y_min = squeeze(min(v(:,2,:))) - margin; y_max = squeeze(max(v(:,2,:))) + margin;
    z_min = squeeze(min(v(:,3,:))) - margin; z_max = squeeze(max(v(:,3,:))) + margin;
    
    in_x = (q(1) >= x_min) & (q(1) <= x_max);
    in_y = (q(2) >= y_min) & (q(2) <= y_max);
    in_z = (q(3) >= z_min) & (q(3) <= z_max);
    
    collision = any(in_x & in_y & in_z);
    feasible = ~collision; 
end

function feasible = collisionChecking(q_new, last_q, v, n)
    feasible = true;
    
    % Controllo ultra-denso: campioniamo la linea ogni 5 metri!
    dist = norm(q_new - last_q);
    steps = max(10, ceil(dist / 5.0)); 
    
    delta_q = (q_new - last_q) / steps;
    q = last_q;
    for i = 1:steps
        q = q + delta_q;
        if ~in_obstacle(q, v, n)
            feasible = false;
            break;
        end
    end
end

function pointIn = InFrame(frame_range, q_rand)
    pointIn = (q_rand(1)>0) && (q_rand(1)<frame_range(1)) && ...
              (q_rand(2)>0) && (q_rand(2)<frame_range(2)) && ...
              (q_rand(3)>0) && (q_rand(3)<frame_range(3));
end

function q_rand = creatPoint(frame_range, last_q, r, v, n, goal_coord)
    r_rand = rand() * r;
    theta = rand() * pi;
    pho = rand() * 2 * pi;
    q_rand = last_q.coord + r_rand * [sin(theta)*cos(pho), sin(theta)*sin(pho), cos(theta)];
    
    if ~in_obstacle(q_rand, v, n) || ~InFrame(frame_range, q_rand)
        q_rand = goal_coord; 
    end
end