function fitness = calc_fitness_pid(X)
    % calc_fitness_pid - 遗传算法适应度函数
    % 功能：计算给定PID参数的控制性能
    % 输入: X = [Kp, Ki, Kd] (1x3向量)
    % 输出: fitness = 适应度值（越大越好）
    
    %% 1. 解包参数
    Kp = X(1);  % 比例增益
    Ki = X(2);  % 积分增益
    Kd = X(3);  % 微分增益
    
    %% 2. 参数有效性检查
    % 确保参数为正数
    if Kp <= 0 || Ki <= 0 || Kd < 0
        fitness = -1e9;  % 返回极差适应度
        return;
    end
    
    %% 3. 设置Simulink模型参数
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
        
        %% 4. 运行仿真
        simOut = sim(modelName, 'StopTime', '100', 'SaveOutput', 'on');
        
        %% 5. 提取仿真数据
        t = simOut.time_data;           % 时间向量
        error = simOut.error_data;      % 误差信号
        output = simOut.output_data;    % 输出信号
        
        %% 6. 计算性能指标
        
        % 6.1 ITAE (Integral of Time-weighted Absolute Error)
        % 优点：对初始误差惩罚小，对后期误差惩罚大，有利于快速响应
        itae = trapz(t, t .* abs(error));
        
        % 6.2 IAE (Integral of Absolute Error)
        % 总体误差积分
        iae = trapz(t, abs(error));
        
        % 6.3 ISE (Integral of Squared Error)
        % 对大误差的惩罚更重
        ise = trapz(t, error.^2);
        
        % 6.4 超调量 (Overshoot)
        % 计算输出相对于设定值(1.0)的超调
        max_output = max(output);
        overshoot_percent = (max_output - 1.0) * 100;
        
        % 超调惩罚
        if overshoot_percent > 5  % 如果超调超过5%
            overshoot_penalty = overshoot_percent * 10;
        else
            overshoot_penalty = 0;
        end
        
        % 6.5 上升时间 (Rise Time)
        % 从10%到90%设定值的时间
        idx_10 = find(output >= 0.1, 1, 'first');
        idx_90 = find(output >= 0.9, 1, 'first');
        if ~isempty(idx_10) && ~isempty(idx_90)
            rise_time = t(idx_90) - t(idx_10);
        else
            rise_time = 100;  % 如果无法达到，惩罚
        end
        
        % 6.6 调节时间 (Settling Time)
        % 进入并保持在±2%误差带内的时间
        settling_band = 0.02;  % ±2%
        settled_indices = find(abs(error) < settling_band);
        if ~isempty(settled_indices)
            % 找到最后一次离开误差带后的首次进入点
            settling_idx = settled_indices(end);
            for i = settled_indices(end):-1:1
                if abs(error(i)) >= settling_band
                    settling_idx = i + 1;
                    break;
                end
            end
            settling_time = t(settling_idx);
        else
            settling_time = 100;  % 惩罚
        end
        
        % 6.7 稳态误差 (Steady-State Error)
        % 取最后10%数据点的平均误差
        n_points = length(error);
        steady_start = floor(0.9 * n_points);
        steady_state_error = abs(mean(error(steady_start:end)));
        
        % 6.8 控制信号能量（防止过度控制）
        % 虽然没有直接记录控制信号，但可以从误差变化推断
        control_effort = sum(abs(diff(error)));
        
        %% 7. 综合性能指标（加权）
        % 根据实际需求调整权重
        w_itae = 1.0;      % ITAE权重
        w_overshoot = 2.0;  % 超调惩罚权重
        w_settling = 0.5;   % 调节时间权重
        w_steady = 50.0;    % 稳态误差权重
        
        % 总成本（越小越好）
        total_cost = w_itae * itae + ...
                     w_overshoot * overshoot_penalty + ...
                     w_settling * settling_time + ...
                     w_steady * steady_state_error;
        
        %% 8. 转换为适应度（GA求最大值）
        if total_cost > 0
            fitness = 1 / total_cost;
        else
            fitness = 1e9;  % 理想情况
        end
        
        % 额外检查：如果系统不稳定（输出发散），给予极差适应度
        if max(abs(output)) > 10
            fitness = -1e9;
        end
        
    catch ME
        % 仿真出错时返回极差适应度
        fprintf('⚠ 仿真出错: %s\n', ME.message);
        fprintf('  参数: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', Kp, Ki, Kd);
        fitness = -1e9;
    end
end
