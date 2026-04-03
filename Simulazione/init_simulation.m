% =========================================================================
% 0. CARICHIAMO I DATI DELLA TRAIETTORIA
% =========================================================================
load('traiettoria_finale.mat'); 
disp('Traiettoria caricata con successo.');

% =========================================================================
% 1. LE CHIAVI LOGICHE
% =========================================================================
plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;  

% =========================================================================
% 2. GESTIONE DATA DICTIONARY E CONDIZIONI INIZIALI
% =========================================================================
disp('Apertura del Data Dictionary di Simulink...');
dict = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');
sect = getSection(dict, 'Design Data');

% --- AGGIORNAMENTO UFFICIALE DELLE CONDIZIONI INIZIALI ---
% Andiamo a modificare il punto di spawn direttamente nella "cassaforte"
entry = getEntry(sect, 'initialConditions');
initStruct = getValue(entry);

% Estraiamo il primo punto [X, Y, Z]
start_point = ground_truth_trajectory(1, :);

% Lo convertiamo in NED (invertendo il segno della Z) e mantenendo il tipo di dato originale
initStruct.posNED = cast([start_point(1), start_point(2), -start_point(3)], class(initStruct.posNED));

setValue(entry, initStruct);
saveChanges(dict);
disp(['✅ Condizioni iniziali aggiornate nel file SLDD! Il drone spawnerà a (NED): [', num2str(initStruct.posNED), ']']);

% --- ESTRAZIONE PARAMETRI E BUS NEL WORKSPACE ---
% Ora estraiamo i parametri fisici (che conterranno l'initialConditions aggiornato)
exportToFile(sect, 'temp_parametri_drone.mat');
load('temp_parametri_drone.mat');

try
    sect_interfaces = getSection(dict, 'Interfaces');
    exportToFile(sect_interfaces, 'temp_buses_drone.mat');
    load('temp_buses_drone.mat');
    disp('Definizioni dei Bus caricate con successo!');
catch
    disp('Nessuna sezione Interfaces trovata. I Bus potrebbero essere nel Base Workspace.');
end
close(dict); % Chiudiamo il dizionario per sicurezza
disp('SETUP COMPLETO! Tutte le chiavi, i parametri e i Bus sono nel Workspace.');

% =========================================================================
% 3. CREAZIONE DELLE TIMESERIES PER IL CONTROLLORE (MINIMUM SNAP)
% =========================================================================
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

disp('-------------------------------------------------------------------');
disp('🚀 Dati formattati per Simulink!');
disp('Le variabili sim_pos_des, sim_vel_des e sim_yaw_des sono pronte.');
disp(['-> ⚠️ TEMPO DA INSERIRE IN SIMULINK (In alto al posto di 10.0): ', num2str(t_sim(end))]);
disp('Ora puoi premere RUN sul modello Simulink.');