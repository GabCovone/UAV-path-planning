function feasible = collisionChecking(q_new, last_q, v, n, margin, step_size)
    feasible = true;
    dist = norm(q_new - last_q);
    steps = max(10, ceil(dist / step_size)); 
    
    delta_q = (q_new - last_q) / steps;
    q = last_q;
    for i = 1:steps
        q = q + delta_q;
        if is_colliding(q, v, margin)
            feasible = false;
            break;
        end
    end
end