function [state, options, optchanged] = recordGAHistory(options, state, flag)
% recordGAHistory - 记录GA优化过程中的收敛历史
% 作为GA的OutputFcn使用

    global ga_history;
    
    optchanged = false;
    
    switch flag
        case 'init'
            % 初始化历史记录
            ga_history.generation = [];
            ga_history.best = [];
            ga_history.mean = [];
            
        case 'iter'
            % 每代记录最优值和平均值
            gen = state.Generation;
            best_val = state.Best(end);
            mean_val = mean(state.Score);
            
            ga_history.generation = [ga_history.generation; gen];
            ga_history.best = [ga_history.best; best_val];
            ga_history.mean = [ga_history.mean; mean_val];
            
        case 'done'
            % 优化完成
            fprintf('GA历史记录完成，共记录 %d 代\n', length(ga_history.generation));
    end
end
