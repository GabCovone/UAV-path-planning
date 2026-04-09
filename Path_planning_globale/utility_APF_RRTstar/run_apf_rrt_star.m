function [nodes, node_count, success] = run_apf_rrt_star(q_start, q_goal, v, n_collision, v_center, v_r, frame_range, params)
    % Preallocazione della memoria
    nodes = repmat(q_start, 1, params.numNodes + 500); 
    nodes(1) = q_start;
    node_count = 1;
    success = false;
    
    for i = 1:params.numNodes
        q_rand = creatPoint(frame_range, nodes(node_count), params.r_create, v, n_collision, q_goal.coord, params.margin);
        
        % Vettorizzazione: Ricerca istantanea del nodo più vicino
        coords_mat = reshape([nodes(1:node_count).coord], 3, node_count)'; 
        dists_to_rand = sqrt(sum((coords_mat - q_rand).^2, 2));
        [~, idx] = min(dists_to_rand);
        q_near = nodes(idx);
        
        % APF (Attrazione verso il goal + Repulsione dagli ostacoli)
        q_new_coord = steer3d(q_rand, q_near.coord, q_goal.coord, params.pho, params.k_att, frame_range, params.k_rep, v_center, v_r, n_collision);
        
        % Controllo Collisioni
        if collisionChecking(q_new_coord, q_near.coord, v, n_collision, params.margin, params.step_size)
            q_new.coord = q_new_coord;
            q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
            q_new.parent = idx;
            
            % Vettorizzazione: Rewiring
            dists_to_new = sqrt(sum((coords_mat - q_new.coord).^2, 2));
            idx_near = find(dists_to_new <= params.r_rewire)';
            
            C_min = q_new.cost;
            
            for j = idx_near
                cost_candidate = nodes(j).cost + dist_3d(nodes(j).coord, q_new.coord);
                if (cost_candidate < C_min) && collisionChecking(q_new.coord, nodes(j).coord, v, n_collision, params.margin, params.step_size)
                    C_min = cost_candidate;
                    q_new.parent = j;
                    q_new.cost = C_min;
                end
            end
            
            % Salvataggio del nuovo nodo
            node_count = node_count + 1;
            nodes(node_count) = q_new;
            
            % Controllo arrivo
            if dist_3d(q_new.coord, q_goal.coord) < params.goal_tol
                disp('🎯 Traguardo raggiunto!');
                success = true;
                break;
            end
        end
    end
    disp(['Nodi generati: ', num2str(node_count)]);
    disp(['Costo percorso grezzo: ', num2str(nodes(node_count).cost)]);
end

function q_rand = creatPoint(frame_range, last_q, r, v, n, goal_coord, margin)
    r_rand = rand() * r;
    theta = rand() * pi;
    pho = rand() * 2 * pi;
    q_rand = last_q.coord + r_rand * [sin(theta)*cos(pho), sin(theta)*sin(pho), cos(theta)];
    
    if is_colliding(q_rand, v, margin) || ~InFrame(frame_range, q_rand)
        q_rand = goal_coord; 
    end
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

function pointIn = InFrame(frame_range, q_rand)
    pointIn = (q_rand(1)>0) && (q_rand(1)<frame_range(1)) && ...
              (q_rand(2)>0) && (q_rand(2)<frame_range(2)) && ...
              (q_rand(3)>0) && (q_rand(3)<frame_range(3));
end

