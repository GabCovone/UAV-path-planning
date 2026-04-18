function visualizza_voxel(voxel_flat, pos_drone, res, r)
    % voxel_flat: vettore 1x1000 dei voxel (0 libero, 1 occupato)
    % pos_drone: coordinate [X, Y, Z] del drone
    % res: risoluzione del voxel (default 5.0m)
    % r: raggio in numero di voxel (default 5, per un cubo 10x10x10)
    
    if nargin < 3, res = 5.0; end
    if nargin < 4, r = 5; end

    % Rimodelliamo il vettore piatto nel cubo 3D [10x10x10]
    cubo = reshape(voxel_flat, [2*r, 2*r, 2*r]);
    
    % Troviamo gli indici 3D (i,j,k) dei soli voxel occupati (valore == 1)
    [I, J, K] = ind2sub(size(cubo), find(cubo > 0));
    num_voxels = length(I);

    % Setup della Figura
    figure('Name', 'Vista Radar (Voxel)', 'Color', 'w', 'Position', [150, 150, 700, 600]);
    hold on; grid on; view(30, 30); axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(sprintf('Percezione Agente: %d / %d Voxel Occupati', num_voxels, (2*r)^3));

    % Disegna il Drone al centro
    plot3(pos_drone(1), pos_drone(2), pos_drone(3), 'kp', 'MarkerSize', 15, ...
          'MarkerFaceColor', 'y', 'DisplayName', 'Posizione Drone');

    % Disegna il "Raggio Visivo" (Il cubo di confine della percezione)
    margin = r * res;
    lim_min = pos_drone(:) - margin;
    lim_max = pos_drone(:) + margin;
    axis([lim_min(1) lim_max(1) lim_min(2) lim_max(2) lim_min(3) lim_max(3)]);
    
    % Se non c'è nessun ostacolo, esci
    if num_voxels == 0
        disp('Nessun ostacolo nel raggio visivo del drone.');
        legend('Location', 'best');
        return;
    end

    % --- CALCOLO COORDINATE REALI E MEGA-PATCH ---
    
    % Ricostruiamo la griglia spaziale esatta usata dal sensore
    vx_range = pos_drone(1) - (r-0.5)*res : res : pos_drone(1) + (r-0.5)*res;
    vy_range = pos_drone(2) - (r-0.5)*res : res : pos_drone(2) + (r-0.5)*res;
    vz_range = pos_drone(3) - (r-0.5)*res : res : pos_drone(3) + (r-0.5)*res;

    % Pre-allochiamo le matrici per il Mega-Patch
    V_tot = zeros(num_voxels * 8, 3);
    F_tot = zeros(num_voxels * 6, 4);

    % Vertici base di un singolo cubo (voxel) centrato in [0,0,0]
    h = res / 2; % semi-lato del cubo
    base_v = [-h -h -h; h -h -h; h h -h; -h h -h; -h -h h; h -h h; h h h; -h h h];
    base_f = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];

    for v = 1:num_voxels
        % Estraiamo il centro reale del voxel v-esimo
        centro_x = vx_range(I(v));
        centro_y = vy_range(J(v));
        centro_z = vz_range(K(v));
        
        v_offset = (v - 1) * 8;
        f_offset = (v - 1) * 6;

        % Trasliamo i vertici base nella posizione corretta
        V_tot(v_offset+1 : v_offset+8, :) = base_v + [centro_x, centro_y, centro_z];

        % Assegniamo le facce
        F_tot(f_offset+1 : f_offset+6, :) = base_f + v_offset;
    end

    % Disegniamo tutti i voxel in un colpo solo (Arancioni semitrasparenti)
    patch('Faces', F_tot, 'Vertices', V_tot, 'FaceColor', [1 0.5 0], ...
          'FaceAlpha', 0.5, 'EdgeColor', [0.8 0.4 0], 'DisplayName', 'Voxel Ostacolo');

    legend('Location', 'best');
    disp('✅ Rendering Voxel completato.');
end