function [statistic, scores, data] = ...
    evaluationFcn(agent, env, trainingInfo)

    persistent logPath
    if isempty(logPath)
        logPathValid_num = evalin('base', 'logPathValid_num');
        logPath = strtrim(char(logPathValid_num));
    end
    
    % Do not use an exploration policy for evaluation.
    agent.UseExplorationPolicy = false;
    
    % Set the number of consecutive evaluation episodes to run.
    numEpisodes = 10;
    
    % Initialize the rewards and data arrays.
    episodeRewards = zeros(numEpisodes, 1);
    data = cell(numEpisodes, 1);

    assignin('base',"is_validation",true);
    
    % Run numEpisodes consecutive evaluation episodes.
    for evaluationEpisode = 1:numEpisodes
    
        % Use a fixed random seed for reproducibility.
        rng(evaluationEpisode*10);
    
        % Run one evaluation episode. The output is a structure
        % containing several agent simulation information,
        % as described in runEpisode.
        episodeResults = runEpisode(env, agent, ...
            MaxSteps=5500, ...
            CleanupPostSim=false);
    
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
    
    % Logging

    fileID = fopen(logPath, 'a');
    fprintf(fileID, ['Rewards: ' repmat(' %1.0f ',1,numel(episodeRewards)) '\n'], episodeRewards);
    fclose(fileID);
    
    % Return the smallest reward
    statistic = median(episodeRewards);
    
    % Reset variables for training
    assignin('base',"is_validation",false);
    agent.UseExplorationPolicy = true;
    
    % Return the rewards vector.
    scores = episodeRewards;

end