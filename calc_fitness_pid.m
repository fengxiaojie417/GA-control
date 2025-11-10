function cost = calc_fitness_pid(X)
    % calc_fitness_pid - 优化的遗传算法成本函数
    % 功能：计算给定PID参数的控制性能，强调系统稳定性
    % 输入: X = [Kp, Ki, Kd] (1x3向量)
    % 输出: cost = 成本值（越小越好）
    
    %% 1. 定义惩罚值
    PENALTY_COST = 1e10;  % 极大惩罚值
    
    %% 2. 解包参数
    Kp = X(1);  % 比例增益
    Ki = X(2);  % 积分增益
    Kd = X(3);  % 微分增益
    
    %% 3. 参数有效性检查
    if Kp <= 0 || Ki < 0 || Kd < 0
        cost = PENALTY_COST;
        return;
    end
    
    %% 4. 参数合理性预检查（避免明显不稳定的组合）
    % 对于二阶系统 G(s)=1/(25s²+10s+1)，过大的Ki会导致不稳定
    if Ki > 2.0 || Kp > 15.0 || Kd > 8.0
        cost = PENALTY_COST * 0.5;  % 轻度惩罚
        return;
    end
    
    %% 5. 设置Simulink模型参数
    modelName = 'pid_tank_model';
    
    try
        % 检查模型是否已加载
        if ~bdIsLoaded(modelName)
            load_system(modelName);
        end
        
        % 设置PID控制器参数
        set_param([modelName '/PID_Controller'], 'P', num2str(Kp));
        set_param([modelName '/PID_Controller'], 'I', num2str(Ki));
        set_param([modelName '/PID_Controller'], 'D', num2str(Kd));
        
        %% 6. 运行仿真
        simOut = sim(modelName, 'StopTime', '100', 'SaveOutput', 'on');
        
        %% 7. 提取仿真数据
        t = simOut.time_data;
        error = simOut.error_data;
        output = simOut.output_data;
        
        %% 8. 系统稳定性检查（多重检查）
        
        % 8.1 检查输出是否发散
        if max(abs(output)) > 5 || any(isnan(output)) || any(isinf(output))
            cost = PENALTY_COST;
            return;
        end
        
        % 8.2 检查后期是否仍在振荡（关键！）
        % 取最后20%的数据检查振荡幅度
        late_start = floor(0.8 * length(output));
        late_output = output(late_start:end);
        late_std = std(late_output);  % 标准差反映振荡程度
        
        % 如果后期标准差过大，说明系统未收敛
        if late_std > 0.05  % 允许±5%的小幅波动
            oscillation_penalty = late_std * 1000;  % 严厉惩罚振荡
            cost = PENALTY_COST * 0.1 + oscillation_penalty;
            return;
        end
        
        % 8.3 检查误差是否持续增大
        late_error = abs(error(late_start:end));
        if max(late_error) > 0.1  % 后期误差不应超过10%
            cost = PENALTY_COST * 0.1;
            return;
        end
        
        %% 9. 计算性能指标
        
        % 9.1 ITAE (Integral of Time-weighted Absolute Error)
        itae = trapz(t, t .* abs(error));
        
        % 9.2 超调量 (Overshoot)
        max_output = max(output);
        overshoot_percent = max(0, (max_output - 1.0) * 100);
        
        % 限制超调量在5%以内，超出部分平方惩罚
        overshoot_limit = 5.0;
        if overshoot_percent > overshoot_limit
            overshoot_penalty = ((overshoot_percent - overshoot_limit) * 2)^2;
        else
            overshoot_penalty = overshoot_percent;  % 小超调线性惩罚
        end
        
        % 9.3 调节时间 (Settling Time) - 2%误差带
        settling_band = 0.02;
        unsettled_indices = find(abs(error) >= settling_band);
        
        if ~isempty(unsettled_indices)
            settling_time = t(unsettled_indices(end));
        else
            settling_time = 0;
        end
        
        % 9.4 稳态误差
        n_points = length(error);
        steady_start = floor(0.9 * n_points);
        steady_state_error = abs(mean(error(steady_start:end)));
        
        % 9.5 上升时间 (10%-90%)
        idx_10 = find(output >= 0.1, 1, 'first');
        idx_90 = find(output >= 0.9, 1, 'first');
        if ~isempty(idx_10) && ~isempty(idx_90)
            rise_time = t(idx_90) - t(idx_10);
        else
            rise_time = 100;  % 如果没有达到，赋予大值
        end
        
        % 9.6 控制信号变化率（平滑性）
        % 计算控制信号u(t) = Kp*e(t) + Ki*∫e + Kd*de/dt
        dt = mean(diff(t));
        control_signal = Kp * error + Ki * cumtrapz(t, error) + Kd * [0; diff(error)/dt];
        control_smoothness = std(diff(control_signal));  % 控制信号的平滑度
        
        %% 10. 综合成本函数（多目标加权）
        
        % 权重设计（根据重要性排序）
        w_stability = 100.0;   % 稳定性（后期波动）- 最重要
        w_steady = 50.0;       % 稳态误差 - 很重要
        w_itae = 1.0;          % ITAE - 重要
        w_overshoot = 20.0;    % 超调惩罚 - 重要
        w_settling = 0.5;      % 调节时间 - 一般
        w_rise = 0.2;          % 上升时间 - 次要
        w_smooth = 0.1;        % 控制平滑性 - 次要
        
        % 稳定性成本（后期标准差）
        stability_cost = late_std * 100;  % 放大后期波动的影响
        
        % 总成本
        total_cost = w_stability * stability_cost + ...
                     w_steady * steady_state_error * 100 + ...
                     w_itae * itae + ...
                     w_overshoot * overshoot_penalty + ...
                     w_settling * settling_time + ...
                     w_rise * rise_time + ...
                     w_smooth * control_smoothness;
        
        cost = total_cost;
        
    catch ME
        % 仿真出错时返回惩罚值
        fprintf('⚠ 仿真出错: %s\n', ME.message);
        fprintf('  参数: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp, Ki, Kd);
        cost = PENALTY_COST;
    end
end