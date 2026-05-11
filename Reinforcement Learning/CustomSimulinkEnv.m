classdef CustomSimulinkEnv < rl.env.SimulinkEnvWithAgent
    properties
        SimulinkModel
        isValidation = false;
    end

    methods
        function env = CustomSimulinkEnv(modelName, agentBlock, obsInfo, actInfo)

           % if nargin < 4
            useFastRestart = 'on';
            %end

            % Initialize parent class
            %env = rlSimulinkEnv(modelName, agentBlock, obsInfo, actInfo);
            env@rl.env.SimulinkEnvWithAgent(modelName, agentBlock, obsInfo, actInfo, useFastRestart);
            
            % Store for later use
            % env.ModelName = modelName;
            % env.AgentBlock = agentBlock;
            % env.ObsInfo = obsInfo;
            % env.ActInfo = actInfo;
        end

        function reset(env)
            is_validation = env.isValidation;
            if is_validation
                reset(env, is_validation);
            else
                reset(env);
            end
        end
        
        function setValidationMode(env, isValidation)
            env.isValidation = isValidation;
        end
    end
end