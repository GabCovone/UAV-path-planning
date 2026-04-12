function [obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo()
    % Definizione costanti del problema
    numVoxels = 1000;
    numState = 10; % 3 vel + 3 omega + 4 quat
    numNominal = 7; % 3 pos + 3 vel + 1 yaw
    numObs = numState + numVoxels + numNominal; % 10 stato + 1000 voxels + 7 nominal = 1017
    numAct = 7; % 3 per posizione, 3 per velocità, 1 per lo yaw
    
    % Spazio delle Osservazioni (Observation Space)
    obsInfo = rlNumericSpec([1 numObs]);
    obsInfo.Name = 'UAV_Observations';
    
    % Spazio delle Azioni (Action Space) :
    % Deviazioni (Delta pos, Delta vel, Delta yaw)
    
    % Limiti massimi sulle deviazioni
    max_delta_pos = 2.0;  % +/- 2 metri
    max_delta_vel = 1.0;  % +/- 1 m/s
    max_delta_yaw = 0.5;  % +/- 0.5 rad
    
    actLimit = [max_delta_pos*ones(3,1); max_delta_vel*ones(3,1); max_delta_yaw];
    actInfo = rlNumericSpec([1 numAct], ...
        'LowerLimit', -actLimit', ...
        'UpperLimit', actLimit');
    actInfo.Name = 'Residual_Actions';
end