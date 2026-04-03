disp('Inizializzazione Generatore Urbano Randomico (8000x8000x1000)...');
tic

% =========================================================================
% 1. PARAMETRI DI BASE E RANDOMIZZAZIONE
% =========================================================================
x_max = 8000; y_max = 8000; z_max = 1000;

% Ogni esecuzione creerà una città e dei punti completamente nuovi
rng('shuffle'); 

% =========================================================================
% 2. GENERAZIONE PROCEDURALE DELLA CITTÀ
% =========================================================================
disp('Costruzione dei grattacieli in corso...');
n_collision = 500;

% Genera la matrice dei vertici della città
[v, creat_center, lengthxyz] = generateCity(n_collision, x_max);

% =========================================================================
% 3. ESTRAZIONE DI START E GOAL CON MARGINE DI SICUREZZA
% =========================================================================
disp('Ricerca di Start e Goal sicuri (Spazio di manovra > 15m)...');

% --- Trova uno START sicuro ---
valido_start = false;
while ~valido_start
    cand_x = 200 + rand() * (x_max - 400);
    cand_y = 200 + rand() * (y_max - 400);
    %cand_z = 20 + rand() * 130; % Quota tra 20m e 150m
    cand_z = 1.0;
    cand_start = [cand_x, cand_y, cand_z];
    
    % Se NON è dentro l'ostacolo (margine 15m incluso), lo accettiamo
    if ~in_obstacle(cand_start, v)
        q_start = cand_start;
        valido_start = true;
    end
end

% --- Trova un GOAL sicuro (e lontano) ---
valido_goal = false;
while ~valido_goal
    cand_x = 200 + rand() * (x_max - 400);
    cand_y = 200 + rand() * (y_max - 400);
    %cand_z = 20 + rand() * 130; 
    cand_z = 1.0;
    cand_goal = [cand_x, cand_y, cand_z];
    
    dist_sg = dist_3d(q_start, cand_goal);
    
    % Deve essere libero da ostacoli (margine 15m) E distante almeno 4000m
    if ~in_obstacle(cand_goal, v) && dist_sg > 4000
        q_goal = cand_goal;
        valido_goal = true;
    end
end

disp(['🟢 Start valido a: [', num2str(q_start, '%.1f '), ']']);
disp(['🟣 Goal valido a : [', num2str(q_goal, '%.1f '), ']']);
disp(['📏 Distanza retta : ', num2str(dist_3d(q_start, q_goal), '%.1f'), ' metri']);

% =========================================================================
% 4. GRAFICA DELLA CITTÀ
% =========================================================================
figure(1); hold on;
title('Mappa Urbana 3D Randomica con Start/Goal Sicuri');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

disp('Rendering 3D in corso...');
for k = 1:n_collision
    V_b = v(:,:,k); 
    f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
    patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
end

% Disegna i marker di Start (Verde) e Goal (Magenta)
plot3(q_start(1), q_start(2), q_start(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(q_goal(1), q_goal(2), q_goal(3), 'mo', 'MarkerSize', 12, 'MarkerFaceColor', 'm');

view(30,30); axis equal; grid on; hold off;
disp(['Tempo totale di generazione: ', num2str(toc), ' secondi']);

% =========================================================================
% 5. SALVATAGGIO DEI DATI
% =========================================================================
save('mappa_urbana.mat', 'v', 'n_collision', 'x_max', 'y_max', 'z_max', 'q_start', 'q_goal');
disp('💾 Mappa e punti strategici salvati in "mappa_urbana.mat".');


% =========================================================================
% FUNZIONI LOCALI
% =========================================================================
function [v, creat_center, lengthxyz] = generateCity(n_collision, max_xy)
    creat_center = zeros(n_collision, 3);
    lengthxyz = zeros(n_collision, 3);
    v = zeros(8, 3, n_collision);
    
    min_pos = 100; max_pos = max_xy - 400; 
    min_len = 100; max_len = 300; 
    margine = 30; 
    
    i = 1;
    while i <= n_collision
        cand_center = [min_pos + (max_pos - min_pos) * rand(1, 2), 0]; 
        cand_len = [min_len + (max_len - min_len) * rand(1, 2), 0];
        
        prob = rand();
        if prob < 0.60
            cand_len(3) = 50 + 100 * rand();       
        elseif prob < 0.90
            cand_len(3) = 150 + 250 * rand();      
        else
            cand_len(3) = 400 + 400 * rand();      
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

function d = dist_3d(q1, q2)
    d = norm(q1 - q2); 
end

function collision = in_obstacle(q, v)
    % MARGINE DI SICUREZZA: 15 metri (per garantire manovre ampie a RRT e Min-Snap)
    margin = 15.0; 
    
    x_min = squeeze(min(v(:,1,:))) - margin; x_max = squeeze(max(v(:,1,:))) + margin;
    y_min = squeeze(min(v(:,2,:))) - margin; y_max = squeeze(max(v(:,2,:))) + margin;
    z_min = squeeze(min(v(:,3,:))) - margin; z_max = squeeze(max(v(:,3,:))) + margin;
    
    in_x = (q(1) >= x_min) & (q(1) <= x_max);
    in_y = (q(2) >= y_min) & (q(2) <= y_max);
    in_z = (q(3) >= z_min) & (q(3) <= z_max);
    
    % Restituisce TRUE se il punto si scontra con l'ostacolo o col suo margine
    collision = any(in_x & in_y & in_z); 
end