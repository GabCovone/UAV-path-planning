function [obsInfo, actInfo, numObs, numAct, actLimit] = get_obsInfo_actInfo()
    % Definizione costanti del problema
    
    numDelays = 3;
    assignin('base', 'delays', numDelays);
    baseNumRays = 256;
    assignin('base', 'num_rays', baseNumRays);
    numRays = baseNumRays * numDelays; % 256 raggi x 3 step (frame stacking)
    numState = 11; % 3 vel + 3 omega + 4 quat + 1 distanza goal
    numErrors = 6; % 3 pos + 3 vel || + 1 yaw
    numObs = numState + numRays + numErrors; % 13 stato, 768 raggi, 6 errori
    numAct = 3; %6; % 3 per posizione, 3 per velocità

    % Spazio delle Osservazioni (Observation Space)
    obsInfo = rlNumericSpec([1 numObs]);
    obsInfo.Name = 'UAV_Observations';
    
    % Spazio delle Azioni (Action Space) :
    % Deviazioni (Delta pos, Delta vel, Delta yaw)
    
    % Limiti massimi sulle deviazioni
    max_delta_pos = 2.0;  % +/- 2 metri
    max_delta_vel = 3.0;  % +/- 3 m/s
    %max_delta_yaw = 0.5;  % +/- 0.5 rad
    
    %actLimit = [max_delta_pos*ones(3,1); max_delta_vel*ones(3,1)];%; max_delta_yaw];

    actLimit = [max_delta_pos*ones(3,1)];%; max_delta_yaw];

    actInfo = rlNumericSpec([1 numAct], ...
        'LowerLimit', -actLimit', ...
        'UpperLimit', actLimit');
    actInfo.Name = 'Residual_Actions';
end