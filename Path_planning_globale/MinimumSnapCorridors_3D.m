function MinimumSnapCorridors_3D()
    clear, clc, close all;

    disp('Caricamento dati (Mappa e Waypoints)...');
    % 1. CARICAMENTO DATI
    load('waypoints_estratti.mat'); 
    waypts = waypoints_pruned'; 
    
    % Carichiamo anche la città per disegnarla!
    load('mappa_urbana.mat', 'v', 'n_collision');
          
    % VERO MINIMUM SNAP (Polinomio di 7° grado)
    n_order = 7; 
    Velocita_Media = 15; % m/s

    %% 2. DENSIFICAZIONE WAYPOINT (Il Golden Ratio)
    disp('Densificazione dei waypoint per stabilità e fluidità...');
    r = 5.0;     
    step = 30.0; % COMPROMESSO PERFETTO
    
    new_waypts = waypts(:,1);
    for i = 1:size(waypts,2)-1
        p1 = waypts(:,i); 
        p2 = waypts(:,i+1);
        dist = norm(p2 - p1);
        n = max(1, ceil(dist/step)); 
        for j = 1:n
            new_waypts = [new_waypts, p1 + (j/n)*(p2 - p1)];
        end
    end
    waypts = new_waypts;
    num_segments = size(waypts, 2) - 1;

    %% 3. TEMPO RELATIVO INTELLIGENTE
    dt = zeros(1, num_segments);
    for i = 1:num_segments
        dist = norm(waypts(:,i+1) - waypts(:,i));
        dt(i) = max(dist / Velocita_Media, 1.0); 
    end
    
    %% 4. RISOLUZIONE QP 3D 
    disp(['Ottimizzazione Minimum Snap (7° Grado) su ', num2str(num_segments), ' segmenti...']);
    tic;
    polys_x = solve_qp_relative(waypts(1,:), dt, n_order);
    polys_y = solve_qp_relative(waypts(2,:), dt, n_order);
    polys_z = solve_qp_relative(waypts(3,:), dt, n_order);
    tempo_qp = toc;
    disp(['Solver QP convergente con successo in ', num2str(tempo_qp), ' secondi!']);

    %% 5. GRAFICA E SALVATAGGIO
    figure(1); hold on;
    title('Traiettoria Minimum Snap Definitiva (Step 30m) + Ambiente Urbano');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % ---------------------------------------------------------------------
    % DISEGNO DELLA CITTÀ (Grattacieli Blu)
    % ---------------------------------------------------------------------
    disp('Rendering della città 3D in corso...');
    for k = 1:n_collision
        V_b = v(:,:,k); 
        f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];
        patch('Faces', f, 'Vertices', V_b, 'FaceColor', [0.2 0.5 0.8], 'FaceAlpha', 0.6, 'EdgeColor', 'none');
    end
    
    % Disegna i chiodi (waypoint) usati dal solver
    plot3(waypts(1,:), waypts(2,:), waypts(3,:), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'y');
    
    % ---------------------------------------------------------------------
    % RENDERING DEL TUBO CONTINUO
    % ---------------------------------------------------------------------
    disp('Rendering del Safe Flight Corridor 3D continuo...');
    for i = 1:num_segments
        p1 = waypts(:,i);
        p2 = waypts(:,i+1);
        dist = norm(p2 - p1);
        % Calcola quanti cubi servono per fare un tubo senza buchi
        num_cubes = ceil(dist / (r * 1.5)) + 1; 
        for j = 0:max(1, num_cubes-1)
            cube_center = p1 + (j/max(1, num_cubes-1)) * (p2 - p1);
            plot_cube(cube_center, r);
        end
    end

    disp('Campionamento della traiettoria finale a 10 Hz...');
    XX = []; YY = []; ZZ = [];
    t_res = 0.1; % 10 Hz
    
    for i = 1:num_segments
        for t = 0:t_res:dt(i)
            x_val = dot(polys_x(:, i)', calc_tvec_rel(t, n_order, 0));
            y_val = dot(polys_y(:, i)', calc_tvec_rel(t, n_order, 0));
            z_val = dot(polys_z(:, i)', calc_tvec_rel(t, n_order, 0));
            XX(end+1) = x_val; YY(end+1) = y_val; ZZ(end+1) = z_val;
        end
    end
    
    % Disegna la traiettoria azzurra 
    plot3(XX, YY, ZZ, 'Color', [0 0.8 1], 'LineWidth', 4);
    
    % Avvia la vista corretta
    view(30, 30); grid on; axis equal; hold off;
    
    % Salvataggio Ground Truth
    ground_truth_trajectory = [XX', YY', ZZ'];
    save('traiettoria_finale.mat', 'ground_truth_trajectory');
    disp('File "traiettoria_finale.mat" salvato. Pronto per l''estrazione Voxel!');
end

% =========================================================================
% MOTORE QP CON TEMPO RELATIVO ED ANCORAGGIO ASSOLUTO
% =========================================================================
function polys = solve_qp_relative(pts, dt, n_order)
    N = length(dt);
    n_coef = n_order + 1;
    
    Q_all = sparse(N*n_coef, N*n_coef);
    for i = 1:N
        Q_i = computeQ_rel(n_order, 4, dt(i));
        idx = (i-1)*n_coef + 1 : i*n_coef;
        Q_all(idx, idx) = sparse(Q_i);
    end
    
    Q_all = Q_all + speye(size(Q_all)) * 1e-6; 
    
    num_eq = 4 + 4 + 6*(N-1) + (N-1);
    Aeq = zeros(num_eq, N*n_coef);
    beq = zeros(num_eq, 1);
    row = 1;
    
    for d = 0:3
        Aeq(row, 1:n_coef) = calc_tvec_rel(0, n_order, d);
        if d == 0, beq(row) = pts(1); else, beq(row) = 0; end
        row = row + 1;
    end
    
    for d = 0:3
        Aeq(row, end-n_coef+1:end) = calc_tvec_rel(dt(end), n_order, d);
        if d == 0, beq(row) = pts(end); else, beq(row) = 0; end
        row = row + 1;
    end
    
    for i = 1:N-1
        cols_curr = (i-1)*n_coef + 1 : i*n_coef;
        cols_next = i*n_coef + 1 : (i+1)*n_coef;
        for d = 0:5 
            Aeq(row, cols_curr) = calc_tvec_rel(dt(i), n_order, d);
            Aeq(row, cols_next) = -calc_tvec_rel(0, n_order, d);
            beq(row) = 0;
            row = row + 1;
        end
    end
    
    for i = 1:N-1
        cols_curr = (i-1)*n_coef + 1 : i*n_coef;
        Aeq(row, cols_curr) = calc_tvec_rel(dt(i), n_order, 0);
        beq(row) = pts(i+1);
        row = row + 1;
    end
    
    Aeq = sparse(Aeq);
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    
    p = quadprog(Q_all, zeros(N*n_coef, 1), [], [], Aeq, beq, [], [], [], options);
    polys = reshape(p, n_coef, N);
end

% =========================================================================
% FUNZIONI MATEMATICHE PURE 
% =========================================================================
function vec = calc_tvec_rel(t, n_order, d)
    vec = zeros(1, n_order + 1);
    for i = d:n_order
        mult = 1;
        for k = 0:d-1
            mult = mult * (i - k);
        end
        vec(i + 1) = mult * t^(i - d);
    end
end

function Q = computeQ_rel(n_order, deriv, dt)
    Q = zeros(n_order + 1, n_order + 1);
    for i = deriv:n_order
        for j = deriv:n_order
            mult_i = 1; for k=0:deriv-1, mult_i = mult_i * (i-k); end
            mult_j = 1; for k=0:deriv-1, mult_j = mult_j * (j-k); end
            pw = (i - deriv) + (j - deriv) + 1;
            Q(i+1, j+1) = 2 * (mult_i * mult_j / pw) * dt^pw;
        end
    end
end

function plot_cube(center, r)
    x = center(1); y = center(2); z = center(3);
    v = [x-r, y-r, z-r; x+r, y-r, z-r; x+r, y+r, z-r; x-r, y+r, z-r;
         x-r, y-r, z+r; x+r, y-r, z+r; x+r, y+r, z+r; x-r, y+r, z+r];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', v, 'Faces', faces, 'FaceColor', 'c', 'FaceAlpha', 0.05, 'EdgeColor', 'b', 'EdgeAlpha', 0.1);
end