function dyn_obs = genera_ostacoli_dinamici(sim_pos_des, num_dyn_obs)
    % GENERA_OSTACOLI_DINAMICI Crea ostacoli sferici mobili che intersecano la traiettoria.
    % 
    % INPUT:
    %   - sim_pos_des: timeseries della posizione desiderata del drone.
    %   - num_dyn_obs: (Opzionale) Numero di ostacoli da generare (default: 3).
    %
    % OUTPUT:
    %   - dyn_obs: struct contenente p0 (posizione iniziale), v (velocità), 
    %              radius (raggio) per ciascun ostacolo.

    % Se il numero di ostacoli non è specificato, impostalo a 3 di default
    if nargin < 2
        num_dyn_obs = 0;
    end

    if num_dyn_obs == 0
        num_dyn_obs = 1;
        dyn_obs = struct('p0', cell(1, num_dyn_obs), 'v', cell(1, num_dyn_obs), 'radius', cell(1, num_dyn_obs));
        dyn_obs(1).p0 = [0 0 0];
        dyn_obs(1).v = [0 0 0];
        dyn_obs(1).radius = 0;
        return;
    end
    
    % Inizializza la struttura vuota per prestazioni ottimali
    dyn_obs = struct('p0', cell(1, num_dyn_obs), 'v', cell(1, num_dyn_obs), 'radius', cell(1, num_dyn_obs));
    
    num_samples = length(sim_pos_des.Time);
    
    % Controllo di sicurezza: la traiettoria deve avere abbastanza punti
    if num_samples < 10
        warning('Timeseries troppo corta per generare ostacoli dinamici.');
        return;
    end

    for k = 1:num_dyn_obs
        % 1. Scegli un punto a caso sulla traiettoria del drone.
        % Evitiamo di far spawnare ostacoli nel primissimo 20% e nell'ultimo 20% del volo.
        idx_target = randi([round(0.2 * num_samples), round(0.8 * num_samples)]);
        
        punto_impatto = sim_pos_des.Data(idx_target, :);
        tempo_impatto = sim_pos_des.Time(idx_target);
        
        % 2. Decidi da dove parte l'ostacolo.
        % Generiamo una direzione 3D casuale e la normalizziamo.
        direzione_random = randn(1, 3); 
        direzione_random = direzione_random / norm(direzione_random); 
        
        % L'ostacolo parte da 30 metri di distanza dal punto di collisione
        distanza_partenza = 30.0; 
        p0 = punto_impatto - direzione_random * distanza_partenza;
        
        % 3. Calcola la velocità costante necessaria per "scontrarsi" col drone.
        % v = spazio / tempo
        v = (punto_impatto - p0) / tempo_impatto;
        
        % 4. Salva i parametri dell'ostacolo (raggio impostato a 2 metri)
        dyn_obs(k).p0 = p0;
        dyn_obs(k).v = v;
        dyn_obs(k).radius = 2.0; 
    end
end