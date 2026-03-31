% 0. CARICHIAMO I DATI (Fondamentale!)
load('traiettoria_finale.mat'); 

% 1. Creiamo il vettore tempo a 10 Hz
dt = 0.1; 
t_sim = (0:dt:(size(ground_truth_trajectory,1)-1)*dt)';

% Se mancano le velocità, le calcoliamo al volo
if ~exist('ground_truth_velocities', 'var')
    ground_truth_velocities = zeros(size(ground_truth_trajectory, 1), 3);
    for k = 1:size(ground_truth_trajectory, 1)-1
        ground_truth_velocities(k, :) = (ground_truth_trajectory(k+1, :) - ground_truth_trajectory(k, :)) / dt;
    end
    ground_truth_velocities(end, :) = ground_truth_velocities(end-1, :);
end

% 2. Creiamo le TimeSeries per Simulink
sim_pos_des = timeseries(ground_truth_trajectory, t_sim);
sim_vel_des = timeseries(ground_truth_velocities, t_sim);

% 1. Calcoliamo l'angolo di imbardata (Yaw) in radianti usando le velocità X e Y
yaw_dinamico = atan2(ground_truth_velocities(:, 2), ground_truth_velocities(:, 1));

% 2. Usiamo "unwrap" per evitare che il drone faccia giri strani su se stesso quando passa da 180° a -180°
yaw_dinamico = unwrap(yaw_dinamico);

% 3. Creiamo la nuova timeseries per Simulink
sim_yaw_des = timeseries(yaw_dinamico, t_sim); 

disp('Dati formattati per Simulink! Le variabili sim_pos_des, sim_vel_des e sim_yaw_des sono nel Workspace.');
disp(['-> TEMPO DA INSERIRE IN SIMULINK (In alto al posto di 10.0): ', num2str(t_sim(end))]);