function collision = is_colliding(q, v, margin)
    x_min = squeeze(min(v(:,1,:))) - margin; x_max = squeeze(max(v(:,1,:))) + margin;
    y_min = squeeze(min(v(:,2,:))) - margin; y_max = squeeze(max(v(:,2,:))) + margin;
    z_min = squeeze(min(v(:,3,:))) - margin; z_max = squeeze(max(v(:,3,:))) + margin;
    
    in_x = (q(1) >= x_min) & (q(1) <= x_max);
    in_y = (q(2) >= y_min) & (q(2) <= y_max);
    in_z = (q(3) >= z_min) & (q(3) <= z_max);
    
    collision = any(in_x & in_y & in_z);
end