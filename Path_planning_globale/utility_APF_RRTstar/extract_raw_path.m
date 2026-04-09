function [waypoints_raw, nodes, node_count] = extract_raw_path(nodes, node_count, q_goal)
    % Trova il nodo più vicino al traguardo e collega la meta
    D = zeros(1, node_count);
    for j = 1:node_count
        D(j) = dist_3d(nodes(j).coord, q_goal.coord);
    end
    [~, idx] = min(D);
    
    q_goal.parent = idx;
    node_count = node_count + 1;
    nodes(node_count) = q_goal;
    
    % Risaliamo l'albero per ottenere gli indici
    wp_indices_temp = zeros(1, node_count); 
    idx_count = node_count; 
    curr_node = node_count;
    
    while curr_node ~= 0
        wp_indices_temp(idx_count) = curr_node; 
        idx_count = idx_count - 1;
        curr_node = nodes(curr_node).parent;
    end
    
    % Tagliamo gli zeri e creiamo la matrice N x 3
    wp_indices = wp_indices_temp(idx_count + 1 : end);
    waypoints_raw = reshape([nodes(wp_indices).coord], 3, length(wp_indices))';
end