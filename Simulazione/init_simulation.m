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

disp('-------------------------------------------------------------------');

% =========================================================================
% 3. CREAZIONE DELLE TIMESERIES PER IL CONTROLLORE (MINIMUM SNAP)
% =========================================================================
[sim_pos_des, sim_vel_des, sim_yaw_des] = estrai_timeseries(ground_truth_trajectory);

disp('🚀 Dati formattati per Simulink!');
disp(['-> ⚠️ TEMPO DA INSERIRE IN SIMULINK (In alto al posto di 10.0): ', num2str(t_sim(end))]);
disp('Ora puoi premere RUN sul modello Simulink.');