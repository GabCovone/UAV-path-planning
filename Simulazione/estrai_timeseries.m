function [sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory)

    % 3.1 Creiamo il vettore tempo a 10 Hz
    dt = 0.1; 
    t_sim = (0:dt:(size(ground_truth_trajectory,1)-1)*dt)';
    
    % 3.2 Se mancano le velocità, le calcoliamo al volo
    if ~exist('ground_truth_velocities', 'var')
        ground_truth_velocities = zeros(size(ground_truth_trajectory, 1), 3);
        for k = 1:size(ground_truth_trajectory, 1)-1
            ground_truth_velocities(k, :) = (ground_truth_trajectory(k+1, :) - ground_truth_trajectory(k, :)) / dt;
        end
        ground_truth_velocities(end, :) = ground_truth_velocities(end-1, :);
    end
    
    % 3.3 Creiamo le TimeSeries di Posizione e Velocità
    sim_pos_des = timeseries(ground_truth_trajectory, t_sim);
    sim_vel_des = timeseries(ground_truth_velocities, t_sim);
    
    % 3.4 Calcoliamo l'angolo di imbardata (Yaw) in radianti usando le velocità X e Y
    yaw_dinamico = atan2(ground_truth_velocities(:, 2), ground_truth_velocities(:, 1));
    
    % 3.5 Usiamo "unwrap" per evitare che il drone faccia giri su se stesso
    yaw_dinamico = unwrap(yaw_dinamico);
    
    % 3.6 Creiamo la timeseries per lo Yaw
    sim_yaw_des = timeseries(yaw_dinamico, t_sim);
    
    disp('Le variabili sim_pos_des, sim_vel_des e sim_yaw_des sono pronte.');
end