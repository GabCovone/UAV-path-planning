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

% Imbardata fissa a 0 (Il drone non ruota su se stesso)
sim_yaw_des = timeseries(zeros(length(t_sim), 1), t_sim); 

disp('Dati formattati per Simulink! Le variabili sim_pos_des, sim_vel_des e sim_yaw_des sono nel Workspace.');
disp(['-> TEMPO DA INSERIRE IN SIMULINK (In alto al posto di 10.0): ', num2str(t_sim(end))]);