function [v, n_collision, q_start_struct, q_goal_struct, frame_range, v_center, v_r] = init_environment(map)
    % Carica la città e i punti strategici generati
    % Se è una stringa allora lo si considera un file con i dati, altrimenti lo si
    % considera pari ai dati
    if isstring(map) || ischar(map)
        load(map, 'v', 'n_collision', 'x_max', 'y_max', 'z_max', 'q_start', 'q_goal');
    elseif isstruct(map)
        v = map.v;
        n_collision = map.n_collision;
        x_max = map.x_max;
        y_max = map.y_max;
        z_max = map.z_max;
        q_start = map.q_start;
        q_goal = map.q_goal;
    else
        error('map deve essere un nome file (stringa) o una struct.');
    end

    frame_range = [x_max, y_max, z_max];
    
    q_start_struct.coord = q_start; 
    q_start_struct.cost = 0;
    q_start_struct.parent = 0;
    
    q_goal_struct.coord = q_goal; 
    q_goal_struct.cost = 0;
    q_goal_struct.parent = 0;
    
    % Calcolo centri e raggi degli ostacoli
    exp_center.coord = [0 0 0];
    v_center = repmat(exp_center, 1, n_collision);
    v_r = zeros(1, n_collision);
    
    for num = 1:n_collision
        v_center(num).coord = [mean([min(v(:,1,num)), max(v(:,1,num))]), ...
                               mean([min(v(:,2,num)), max(v(:,2,num))]), ...
                               mean([min(v(:,3,num)), max(v(:,3,num))])];
        v_r(num) = dist_3d(v(1,:,num), v_center(num).coord);
    end
end