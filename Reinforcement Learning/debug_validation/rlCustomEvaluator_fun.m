% Versione modificata della funzione rlCustomEvaluator
% per effettuare debugging sulla funzione di evaluation personalizzata

function evaluator = rlCustomEvaluator_fun(EvaluationFcn, NameValueArgs)
% RLCUSTOMEVALUATOR : Creates a custom evaluator object for evaluating reinforcement learning agents during training.
%
% evaluator = rlCustomEvaluator(EVALUATIONFCN) returns a custom evaluator object. The EVALUATIONFCN argument is a handle to your
% custom MATLAB evaluation function.
%
% evaluator = rlCustomEvaluator(EVALUATIONFCN, "EvaluationFrequency", EVALUATIONFREQUENCY ) creates a custom evaluator
% object by specifying the evaluation frequency.
%
%     EVALUATIONFCN : Custom evaluation function, specified as a function handle. The train function calls EVALUATIONFCN
%                     after EVALUATIONFREQUENCY episodes. Your evaluation function must have three inputs and three outputs, as
%                     illustrated by the following signature.
%
%                     [STATISTIC, EVALUATIONSCORES, EVALUATIONDATA] = myEvalFcn(AGENT, ENV, TRAININGEPISODEINFO)
%
%                     Given an agent, its environment, and training episode information, the custom evaluation function runs a
%                     number of evaluation episodes and returns a corresponding summarizing statistics, a vector of episode
%                     scores, and any additional data that might be needed for logging.
%
%  The required input arguments (passed to EVALUATIONFCN from train), are described as follows.
%     AGENT               : Agent to evaluate, specified as a reinforcement learning agent object. For multiagent environments,
%                           this is a cell array of agent objects.
%     ENV                 : Environments within which the agents are evaluated, specified as a reinforcement environment object.
%     TRAININGEPISODEINFO : A structure containing the following fields.
%         EpisodeIndex    : Current episode index, specified as a positive integer.
%         EpisodeInfo     : A structure containing the fields CumulativeReward, StepsTaken, and InitialObservation, which contain,
%                           respectively, the cumulative reward, the number of steps taken, and the initial observations of the
%                           current training episode.
%
%   The output arguments (passed from evalFcn to train), are described as follows.
%     STATISTIC            : A statistic computed from a group of consecutive evaluation episodes. Common statistics are mean,
%                            medium, maximum, and minimum value. At the end of the training, this value is returned by train as
%                            the element of the EvaluationStatistic vector.
%     EVALUATIONSCORES     : A vector of episode scores from each evaluation episode. You can use a logger object to store this
%                            argument during training.
%     EVALUATIONDATA       : Any additional data from evaluation that you might find useful, for example for logging purposes. You
%                             can use a logger object to store this argument during training.
%
%   Example 1: Run 1 evaluation episode until the 1000th training episode, and run 10 evaluation
%   episodes after that. Evaluation is performed at every 100 training episodes.
%   Agent does not use any exploration. The median of evaluation episodes is used as a statistic.
%
%   % Create an evaluator
%   evaluator = rlCustomEvaluator(@myEvaluationFcn, EvaluationFrequency = 100)
%
%   % Evaluate agent during training
%   results = train(agent, env, options, Evaluator=evaluator)
%
% function  [statistic, evaluationEpisodeScores, evaluationData] = myEvaluationFcn(agent, env, trainingEpisodeInfo)
%     agent.UseExplorationPolicy = false;
%     if trainingEpisodeInfo.EpisodeIndex <= 1000
%         numEpisodes = 1;
%     else
%         numEpisodes = 10;
%     end
% 
%     episodeRewards = zeros(numEpisodes, 1);
%     evaluationData = cell(numEpisodes, 1);
% 
%     % Determine whether the environment is configured for parallel simulations.
%     simData = getSimData(env);
%     if ~isempty(simData)
%         useParallel = simData.UseParallel;
%     else
%         useParallel = false;
%     end
% 
%     if useParallel
%         data = rl.env.Future.empty(numEpisodes,0);
%     end
% 
%     for evaluationEpisode = 1:numEpisodes
%         % Use a fixed random seed
%         rng(evaluationEpisode*10)
%         result = runEpisode(env, agent, MaxSteps = 500, CleanupPostSim = false);
%         if useParallel
%             data(evaluationEpisode) = result;
%         else
%             episodeRewards(evaluationEpisode) = result.AgentData.EpisodeInfo.CumulativeReward;
%             evaluationData{evaluationEpisode} = result;
%         end
%     end
% 
%     if useParallel
%         % Use fetchOutputs to wait until all remaining simulations are completed and then retrieve all outputs.
%         outs = fetchOutputs(data);
%         for evaluationEpisode = 1:numEpisodes
%             episodeRewards(evaluationEpisode) = outs(evaluationEpisode).AgentData.EpisodeInfo.CumulativeReward;
%         end
%         delete(data);
%     end
% 
%     statistic = median(episodeRewards);
%     evaluationEpisodeScores = episodeRewards;
% end
%
% See also: rlEvaluator, train

% Copyright 2023 The MathWorks, Inc.

    arguments
        EvaluationFcn function_handle
        NameValueArgs.EvaluationFrequency (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 100;
    end
    evaluator = rlCustomEvaluator(EvaluationFcn, NameValueArgs);
end
