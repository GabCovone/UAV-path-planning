function lidar_scan = estrai_lidar_scan(pos_drone, bounds, dyn_obs, t, num_rays)
    % Parametri del sensore
    max_range = 20.0; % Raggio massimo in metri
    
    % === 1. GENERAZIONE DIREZIONI RAGGI (SFERA DI FIBONACCI) ===
    indices = 0:(num_rays - 1);
    phi = pi * (3 - sqrt(5)); % Angolo aureo in radianti
    
    z = 1 - (indices ./ (num_rays - 1)) .* 2; 
    raggio_xy = sqrt(1 - z.^2); 
    theta = phi .* indices;      
    
    x = cos(theta) .* raggio_xy;
    y = sin(theta) .* raggio_xy;
    
    dirs = double([x; y; z]); 
    
    lidar_scan = max_range * ones(1, num_rays, 'double');
    pos_drone = double(pos_drone);

    % === 1.5 ADATTAMENTO FORMATO BOUNDS (STRUCT o MATRICE) ===
    % Questo blocco rende la funzione universale per qualsiasi tipo di mappa
    if isstruct(bounds)
        b_xmin = bounds.x_min; b_xmax = bounds.x_max;
        b_ymin = bounds.y_min; b_ymax = bounds.y_max;
        b_zmin = bounds.z_min; b_zmax = bounds.z_max;
    else
        % Se bounds è una matrice Nx6 (es. map.v)
        if size(bounds, 2) >= 6
            % Controlla se il formato è [xmin, xmax, ymin, ymax, ...] o [xmin, ymin, zmin, xmax, ...]
            if bounds(1,2) > bounds(1,1) && bounds(1,4) > bounds(1,3)
                b_xmin = bounds(:,1); b_xmax = bounds(:,2);
                b_ymin = bounds(:,3); b_ymax = bounds(:,4);
                b_zmin = bounds(:,5); b_zmax = bounds(:,6);
            else
                b_xmin = bounds(:,1); b_xmax = bounds(:,4);
                b_ymin = bounds(:,2); b_ymax = bounds(:,5);
                b_zmin = bounds(:,3); b_zmax = bounds(:,6);
            end
        else
            b_xmin = []; b_xmax = []; b_ymin = []; b_ymax = []; b_zmin = []; b_zmax = [];
        end
    end

    % === 2. INTERSEZIONE CON OSTACOLI STATICI (PALAZZI - SLAB METHOD) ===
    if ~isempty(b_xmin)
        % Uso i vettori estratti al passo 1.5
        vicini_idx = find(~(b_xmax < pos_drone(1)-max_range | b_xmin > pos_drone(1)+max_range | ...
                            b_ymax < pos_drone(2)-max_range | b_ymin > pos_drone(2)+max_range | ...
                            b_zmax < pos_drone(3)-max_range | b_zmin > pos_drone(3)+max_range));
                            
        if ~isempty(vicini_idx)
            inv_dirs = 1 ./ (dirs + 1e-8); 
            
            for i = 1:length(vicini_idx)
                idx = vicini_idx(i);
                
                b_min = [b_xmin(idx); b_ymin(idx); b_zmin(idx)];
                b_max = [b_xmax(idx); b_ymax(idx); b_zmax(idx)];
                
                t0 = (b_min - pos_drone(:)) .* inv_dirs; 
                t1 = (b_max - pos_drone(:)) .* inv_dirs; 
                
                tmin = min(t0, t1); 
                tmax = max(t0, t1); 
                
                t_enter = max(tmin, [], 1); 
                t_exit = min(tmax, [], 1);  
                
                hit_mask = (t_enter <= t_exit) & (t_exit > 0);
                
                if any(hit_mask)
                    hitting_rays = find(hit_mask);
                    for j = 1:length(hitting_rays)
                        r_idx = hitting_rays(j);
                        if t_enter(r_idx) > 0
                            d_imp = t_enter(r_idx);
                        else
                            d_imp = 0; 
                        end
                        if d_imp < lidar_scan(r_idx)
                            lidar_scan(r_idx) = d_imp;
                        end
                    end
                end
            end
        end
    end

    % === 3. INTERSEZIONE CON OSTACOLI DINAMICI (SFERE) ===
    if ~isempty(dyn_obs)
        for k = 1:length(dyn_obs)
            if t < dyn_obs(k).t_start
                continue; % Se il tempo attuale è minore del tempo di nascita, ignora l'ostacolo!
            end


            centro_sfera = dyn_obs(k).p0 + dyn_obs(k).v * t;
            raggio_sfera = dyn_obs(k).radius;
            
            v = centro_sfera(:) - pos_drone(:);
            proj = v' * dirs; 
            valid_idx = proj > 0;
            
            if any(valid_idx)
                distanza_ortogonale_sq = dot(v, v) - proj(valid_idx).^2;
                hit_idx = distanza_ortogonale_sq <= raggio_sfera^2;
                
                if any(hit_idx)
                    actual_rays = find(valid_idx);
                    hitting_rays = actual_rays(hit_idx);
                    
                    proj_hits = proj(hitting_rays);
                    dist_ort_sq_hits = distanza_ortogonale_sq(hit_idx);
                    distanza_impatto = proj_hits - sqrt(raggio_sfera^2 - dist_ort_sq_hits);
                    
                    for i = 1:length(hitting_rays)
                        r_idx = hitting_rays(i);
                        d_imp = distanza_impatto(i);
                        if d_imp > 0 && d_imp < lidar_scan(r_idx)
                            lidar_scan(r_idx) = d_imp;
                        end
                    end
                end
            end
        end
    end
    
    % Normalizzazione
    lidar_scan = double(lidar_scan) / max_range;
end