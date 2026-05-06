% Versione modificata della classe rl.env.rlCustomEvaluator
% per effettuare debugging sulla funzione di evaluation personalizzata

classdef rlCustomEvaluator < rl.evaluation.rlCustomEvaluator
    % rlCustomEvaluator: Custom evaluator

    % Copyright 2023 The MathWorks, Inc.

    methods
        function this = rlCustomEvaluator(evalautionFcn, value)
            this@rl.evaluation.rlCustomEvaluator(evalautionFcn, value);
        end
    end
    methods (Access = protected)
        function [statistic, episodeRewards, episodeData] = evaluate_(this, agent, env, trainingEpisodeInfo)
            % Evaluate agent
            % try
                [statistic, episodeRewards, episodeData] = this.EvaluationFcn(agent, env, trainingEpisodeInfo);
            % catch
            %     error(message('rl:general:EvaluatorErrorCustomFunction'));
            % end
        end
    end
end