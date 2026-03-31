% --- 1. LE CHIAVI LOGICHE ---
plantModelFi = 1;            
useHeading = 1;              
initialGainsMultiplier = 15;  

% --- 2. I PARAMETRI FISICI E I BUS ---
dict = Simulink.data.dictionary.open('uavPackageDeliveryDataDict.sldd');

% Estrai i parametri fisici
sect = getSection(dict, 'Design Data');
exportToFile(sect, 'temp_parametri_drone.mat');
load('temp_parametri_drone.mat');

% Estrai le definizioni dei Bus (FONDAMENTALE PER LE LINEE ROSSE!)
try
    sect_interfaces = getSection(dict, 'Interfaces');
    exportToFile(sect_interfaces, 'temp_buses_drone.mat');
    load('temp_buses_drone.mat');
    disp('Definizioni dei Bus caricate con successo!');
catch
    disp('Nessuna sezione Interfaces trovata. I Bus potrebbero essere nel Base Workspace.');
end

disp('SETUP COMPLETO! Tutte le chiavi, i parametri e i Bus sono nel Workspace.');