function dyn_obs = genera_ostacoli_dinamici_deen(pos_array, time_array, num_dyn_obs, raggi, statici)
    if nargin < 3, num_dyn_obs = 0; end
    if nargin < 4, raggi = [2.0 2.0]; end
    if nargin < 5, statici = "no"; end
    
    isStatic = false; 
    
    if num_dyn_obs == 0
        num_dyn_obs = 1;
        dyn_obs = struct('p0', cell(1, num_dyn_obs), 'v', cell(1, num_dyn_obs), 'radius', cell(1, num_dyn_obs), 't_start', cell(1, num_dyn_obs));
        dyn_obs(1).p0 = [0 0 0]; dyn_obs(1).v = [0 0 0]; dyn_obs(1).radius = 0; dyn_obs(1).t_start = 0;
        return;
    end
    
    min_radius = raggi(1); max_radius = raggi(2);
    dyn_obs = struct('p0', cell(1, num_dyn_obs), 'v', cell(1, num_dyn_obs), 'radius', cell(1, num_dyn_obs), 't_start', cell(1, num_dyn_obs));
    
    num_samples = length(time_array);
    if num_samples < 10
        warning('Traiettoria troppo corta per generare ostacoli dinamici.'); return;
    end
    
    if statici == "si", isStatic = true;
    elseif statici == "no", isStatic = false; end
    
    for k = 1:num_dyn_obs
        % 1. Scegliamo quando far passare l'ostacolo (a metà volo)
        idx_target = randi([round(0.4 * num_samples), round(0.6 * num_samples)]);
        punto_drone = pos_array(idx_target, :);
        
        if statici == "casuale", isStatic = logical(randi([0 1])); end
        
        if ~isStatic
            tempo_impatto = time_array(idx_target);
            
            % 2. L'ostacolo "nasce" 5 secondi prima del passaggio
            tempo_preavviso = 5.0; 
            t_start = max(0, tempo_impatto - tempo_preavviso);
            
            % --- LA MAGIA DEL NEAR MISS ---
            % Creiamo un punto "bersaglio" sfalsato di 2.5 metri dal drone
            % così la sfera attraversa la visuale senza toccare la hitbox del drone.
            dir_sfalsata = randn(1, 3);
            dir_sfalsata = dir_sfalsata / norm(dir_sfalsata);
            punto_bersaglio = punto_drone + dir_sfalsata * (max_radius + 1.0);
            
            direzione_partenza = randn(1, 3); 
            direzione_partenza = direzione_partenza / norm(direzione_partenza); 
            
            % La sfera parte da 40 metri di distanza
            distanza_partenza = 40.0; 
            p0 = punto_bersaglio - direzione_partenza * distanza_partenza;
            
            % Calcoliamo la velocità per colpire il BERSAGLIO SFALSATO
            v = (punto_bersaglio - p0) / (tempo_impatto - t_start);
            
            % Aggiustiamo p0 per l'equazione temporale globale
            p0_globale = p0 - v * t_start; 
        else
            v = [0.0, 0.0, 0.0]; 
            % Anche se statico, lo mettiamo vicino al drone ma non DENTRO al drone
            dir_sfalsata = randn(1, 3); dir_sfalsata = dir_sfalsata / norm(dir_sfalsata);
            p0_globale = punto_drone + dir_sfalsata * (max_radius + 1.5);
            t_start = 0;
        end
        
        dyn_obs(k).p0 = p0_globale;
        dyn_obs(k).v = v;
        dyn_obs(k).radius = min_radius + (max_radius - min_radius) * rand;
        dyn_obs(k).t_start = t_start; 
    end
end