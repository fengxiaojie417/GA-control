function cost = calc_fitness_pid(params)
% calc_fitness_pid - 计算PID参数的适应度值
% 输入: params = [Kp, Ki, Kd]
% 输出: cost = 综合性能指标（越小越好）

    % 模型名称
    modelName = 'pid_tank_model';
    
    % 提取PID参数
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    
    % 参数有效性检查
    if Kp <= 0 || Ki <= 0 || Kd < 0
        cost = 1e10;  % 惩罚无效参数
        return;
    end
    
    try
        % 设置PID参数
        set_param([modelName '/PID_Controller'], 'P', num2str(Kp));
        set_param([modelName '/PID_Controller'], 'I', num2str(Ki));
        set_param([modelName '/PID_Controller'], 'D', num2str(Kd));
        
        % 运行仿真
        simOut = sim(modelName, 'StopTime', '100');
        
        % 提取仿真数据
        t = simOut.time_data;
        error_signal = simOut.error_data;
        output_signal = simOut.output_data;
        
        % 计算性能指标
        
        % 1. ITAE (Integral Time Absolute Error) - 主要指标
        itae = trapz(t, t .* abs(error_signal));
        
        % 2. 超调量惩罚
        overshoot = max(0, max(output_signal) - 1.0);
        overshoot_penalty = 100 * overshoot^2;
        
        % 3. 稳态误差
        steady_state_error = abs(mean(error_signal(floor(0.9*length(error_signal)):end)));
        steady_penalty = 1000 * steady_state_error;
        
        % 4. 调节时间估计（±2%误差带）
        settling_band = 0.02;
        unsettled_indices = find(abs(error_signal) >= settling_band);
        if ~isempty(unsettled_indices)
            settling_time = t(unsettled_indices(end));
        else
            settling_time = t(find(abs(error_signal) < settling_band, 1, 'first'));
        end
        settling_penalty = 0.1 * settling_time;
        
        % 5. 控制信号平滑度（避免剧烈震荡）
        dt = mean(diff(t));
        control_signal = Kp * error_signal + Ki * cumtrapz(t, error_signal) + ...
                        Kd * [0; diff(error_signal)/dt];
        control_smoothness = std(diff(control_signal));
        smoothness_penalty = 0.5 * control_smoothness;
        
        % 6. 后期波动惩罚
        late_output = output_signal(floor(0.8*length(output_signal)):end);
        late_std = std(late_output);
        oscillation_penalty = 500 * late_std;
        
        % 综合成本函数
        cost = itae + overshoot_penalty + steady_penalty + ...
               settling_penalty + smoothness_penalty + oscillation_penalty;
        
        % 防止数值异常
        if isnan(cost) || isinf(cost)
            cost = 1e10;
        end
        
    catch ME
        % 仿真失败，返回大惩罚值
        warning('仿真失败: %s', ME.message);
        cost = 1e10;
    end
end