function [obsInfo, actInfo, numObs, numAct, actLimit, StructNumObs] = get_obsInfo_actInfo()
    % Definizione costanti del problema
    
    numDelays = 3;
    assignin('base', 'delays', numDelays);
    baseNumRays = 256;
    assignin('base', 'num_rays', baseNumRays);
    numRays = baseNumRays * numDelays; % 256 raggi x 3 step (frame stacking)
    numState = 10; % 3 vel + 3 omega + 4 quat
    numErrors = 6; % 3 pos + 3 vel || + 1 yaw
    numObs = numState + numErrors + numRays; % 10 stato + 7 errori + 728 raggi
    StructNumObs = pack_struct(numState, numErrors, numRays); % 10 stato, 728 voxels, 7 errori
    numAct = 6; % 3 per posizione, 3 per velocità
    
    % Spazio delle Osservazioni (Observation Space)

    % 1. Definisci l'elemento del Bus per i Voxel
    voxelElem = Simulink.BusElement;
    voxelElem.Name = 'Rays_Observations'; % DEVE corrispondere al nome del layer di input
    voxelElem.Dimensions = [1, numRays];
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