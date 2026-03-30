function set_up_project()
%set_up_project:  Configure the task for this project
%
%   Set up the task for the current project. This function is set to
%   Run at Startup.
%  Copyright 2021 The MathWorks, Inc.

%% Show the model and its guide
uiopen('uavPIDAutotuning', 1);
load_system('photorealisticSimulationEngi');

if exist('PIDAutotuningMultirotorExample.mlx', 'file') == 2
    edit('PIDAutotuningMultirotorExample.mlx');
end

%% Set directory specific parameters

evalin('base', 'modelInit;'); 

%% Store created variables for cleanup upon exit
evalin('base', 'setupVars = who;');

end
