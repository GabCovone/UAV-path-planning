function waypoints_pruned = prune_path_los(waypoints_raw, v, n_collision, params)
    max_wp = size(waypoints_raw, 1);
    waypoints_pruned_temp = zeros(max_wp, 3); 
    waypoints_pruned_temp(1, :) = waypoints_raw(1, :); 
    
    pruned_count = 1; 
    curr_idx = 1;
    
    while curr_idx < max_wp
        next_idx = max_wp; 
        
        while next_idx > curr_idx + 1
            if collisionChecking(waypoints_raw(next_idx, :), waypoints_raw(curr_idx, :), v, n_collision, params.margin, params.step_size)
                break; 
            end
            next_idx = next_idx - 1; 
        end
        
        pruned_count = pruned_count + 1;
        waypoints_pruned_temp(pruned_count, :) = waypoints_raw(next_idx, :);
        curr_idx = next_idx;
    end
    
    waypoints_pruned = waypoints_pruned_temp(1:pruned_count, :);
    disp(['Waypoint grezzi originali: ', num2str(max_wp)]);
    disp(['Waypoint essenziali (Post-Pruning): ', num2str(pruned_count)]);
end