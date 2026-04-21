function [obsInfo, actInfo, numObs, numAct, actLimit, StructNumObs] = get_obsInfo_actInfo()
    % Definizione costanti del problema
    numVoxels = 1000;
    numState = 10; % 3 vel + 3 omega + 4 quat
    numErrors = 6; % 3 pos + 3 vel || + 1 yaw
    numObs = numState + numErrors + numVoxels; % 10 stato + 7 errori + 1000 voxels = 1017
    StructNumObs = pack_struct(numState, numErrors, numVoxels); % 10 stato, 1000 voxels, 7 errori, tot 1017
    numAct = 6; % 3 per posizione, 3 per velocità ||, 1 per lo yaw
    
    % Spazio delle Osservazioni (Observation Space)

    % 1. Definisci l'elemento del Bus per i Voxel
    voxelElem = Simulink.BusElement;
    voxelElem.Name = 'Voxels_Observations'; % DEVE corrispondere al nome del layer di input
    voxelElem.Dimensions = [10 10 10];
    voxelElem.DataType = 'double';
    
    % 2. Definisci l'elemento del Bus per lo Stato
    stateElem = Simulink.BusElement;
    stateElem.Name = 'State_Observations'; % DEVE corrispondere al nome del layer di input
    stateElem.Dimensions = [numState+numErrors, 1];
    stateElem.DataType = 'double';
    
    % 3. Crea l'oggetto Bus
    ObservationBus = Simulink.Bus;
    ObservationBus.Elements = [voxelElem, stateElem];

    assignin('base', 'ObservationBus', ObservationBus);
    
    clear voxelElem stateElem;

    obsInfo = bus2RLSpec('ObservationBus');
    
    % Spazio delle Azioni (Action Space) :
    % Deviazioni (Delta pos, Delta vel, Delta yaw)
    
    % Limiti massimi sulle deviazioni
    max_delta_pos = 2.0;  % +/- 2 metri
    max_delta_vel = 3.0;  % +/- 3 m/s
    %max_delta_yaw = 0.5;  % +/- 0.5 rad
    
    actLimit = [max_delta_pos*ones(3,1); max_delta_vel*ones(3,1)];%; max_delta_yaw];
    actInfo = rlNumericSpec([1 numAct], ...
        'LowerLimit', -actLimit', ...
        'UpperLimit', actLimit');
    actInfo.Name = 'Residual_Actions';
end