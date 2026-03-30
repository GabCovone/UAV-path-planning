% --- 1. LE CHIAVI LOGICHE ---
plantModelFi = 1;            % Forza l'uso del modello ad alta fedeltà
useHeading = 0;              % Disattiva l'uso della bussola magnetica
initialGainsMultiplier = 1;  % Attiva i moltiplicatori dei PID

% --- 2. I PARAMETRI FISICI (Svuotiamo il forziere) ---
dict = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');
sect = getSection(dict, 'Design Data');
exportToFile(sect, 'temp_parametri_drone.mat');
load('temp_parametri_drone.mat');

disp('SETUP COMPLETO! Tutte le chiavi e i parametri fisici sono nel Workspace.');