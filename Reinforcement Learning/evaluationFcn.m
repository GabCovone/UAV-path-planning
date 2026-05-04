function [statistic, scores, data] = ...
    evaluationFcn(agent, env, trainingInfo)

    disp("debug1")
    
    % Do not use an exploration policy for evaluation.
    agent.UseExplorationPolicy = false;
    
    % Set the number of consecutive evaluation episodes to run.
    numEpisodes = 10;
    
    % Initialize the rewards and data arrays.
    episodeRewards = zeros(numEpisodes, 1);
    data = cell(numEpisodes, 1);
    
    is_validation = true;
    
    % Run numEpisodes consecutive evaluation episodes.
    for evaluationEpisode = 1:numEpisodes
    
        % Use a fixed random seed for reproducibility.
        rng(evaluationEpisode*10)

        reset(env, is_validation);

        disp("debug2")
    
        % Run one evaluation episode. The output is a structure
        % containing several agent simulation information,
        % as described in runEpisode.
        episodeResults = runEpisode(env, agent, ...
            MaxSteps=5500, ...
            CleanupPostSim=false);
        
        disp("debug3")
    
        if isa(episodeResults,"rl.env.Future")
    
            % For parallel simulation, fetch data from workers.
            [~,out] = fetchNext(episodeResults);
    
            % Collect the episode cumulative reward.
            episodeRewards(evaluationEpisode) = ...
                out.AgentData.EpisodeInfo.CumulativeReward;
    
            % Collect the whole data structure.
            data{evaluationEpisode} = out;
    
        else
    
            % Collect the episode cumulative reward.
            episodeRewards(evaluationEpisode) = ...
                episodeResults.AgentData.EpisodeInfo.CumulativeReward;
            data{evaluationEpisode} = episodeResults;
        end
    end
    
    % Imposta di nuovo gli scenari da usare a quelli di training
    assignin('base', 'path_DB_scenari', path_DB_scenari);
    
    % Return the smallest reward
    statistic = min(episodeRewards);

    disp("debug4")
    
    % Return the rewards vector.
    scores = episodeRewards;

end