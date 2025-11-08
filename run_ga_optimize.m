%% run_ga_optimize.m - 遗传算法优化PID参数主程序
% 功能：使用遗传算法优化双容水箱PID控制器参数

clear; clc; close all;

fprintf('========================================\n');
fprintf('遗传算法优化PID控制器\n');
fprintf('========================================\n\n');

%% 1. 初始化
modelName = 'pid_tank_model';

% 加载Simulink模型
if ~bdIsLoaded(modelName)
    load_system(modelName);
    fprintf('✓ 已加载模型: %s\n', modelName);
else
    fprintf('✓ 模型已在内存中\n');
end

%% 2. 设置GA参数
fprintf('\n配置遗传算法参数...\n');

n_vars = 3;  % 优化变量个数 [Kp, Ki, Kd]

% 参数搜索边界
%        Kp    Ki    Kd
lb = [  0.1,  0.01,  0.0 ];  % 下限
ub = [ 20.0, 10.0,  10.0 ];  % 上限

fprintf('  变量个数: %d\n', n_vars);
fprintf('  Kp范围: [%.2f, %.2f]\n', lb(1), ub(1));
fprintf('  Ki范围: [%.2f, %.2f]\n', lb(2), ub(2));
fprintf('  Kd范围: [%.2f, %.2f]\n', lb(3), ub(3));

%% 3. 配置GA选项
fprintf('\n配置GA选项...\n');

options = optimoptions('ga', ...
    'PopulationSize', 50, ...              % 种群大小
    'MaxGenerations', 40, ...              % 最大代数
    'EliteCount', 5, ...                   % 精英个体数
    'CrossoverFraction', 0.8, ...          % 交叉概率
    'FunctionTolerance', 1e-6, ...         % 函数容差
    'MutationFcn', {@mutationadaptfeasible}, ...  % 自适应变异
    'SelectionFcn', @selectionroulette, ...       % 轮盘赌选择
    'Display', 'iter', ...                 % 显示每代信息
    'PlotFcn', {@gaplotbestf, @gaplotbestindiv, @gaplotexpectation}, ... % 绘图
    'UseParallel', false);                 % 如有并行工具箱可改为true

fprintf('  种群大小: %d\n', options.PopulationSize);
fprintf('  最大代数: %d\n', options.MaxGenerations);
fprintf('  精英保留: %d\n', options.EliteCount);
fprintf('  交叉概率: %.2f\n', options.CrossoverFraction);

%% 4. 运行遗传算法
fprintf('\n========================================\n');
fprintf('开始遗传算法优化...\n');
fprintf('========================================\n\n');

tic;  % 开始计时

% 执行GA优化
% @calc_fitness_pid 是适应度函数
% [] [] [] [] 表示没有线性不等式、等式约束
% [] 表示没有非线性约束
[best_params, best_fitness] = ga(@calc_fitness_pid, n_vars, ...
                                  [], [], [], [], ...  % 无线性约束
                                  lb, ub, ...          % 边界约束
                                  [], ...              % 无非线性约束
                                  options);

elapsed_time = toc;  % 结束计时

%% 5. 显示优化结果
fprintf('\n========================================\n');
fprintf('优化完成！\n');
fprintf('========================================\n');
fprintf('总耗时: %.2f 秒 (%.2f 分钟)\n\n', elapsed_time, elapsed_time/60);

fprintf('最优适应度值: %.6f\n', best_fitness);
fprintf('估计成本值: %.4f\n\n', 1/best_fitness);

fprintf('最优PID参数:\n');
fprintf('  Kp = %.6f\n', best_params(1));
fprintf('  Ki = %.6f\n', best_params(2));
fprintf('  Kd = %.6f\n', best_params(3));

%% 6. 保存优化结果
save('ga_pid_results.mat', 'best_params', 'best_fitness', 'elapsed_time');
fprintf('\n✓ 结果已保存到: ga_pid_results.mat\n');

%% 7. 用最优参数进行验证仿真
fprintf('\n========================================\n');
fprintf('验证仿真\n');
fprintf('========================================\n');

% 设置最优参数
set_param([modelName '/PID_Controller'], 'P', num2str(best_params(1)));
set_param([modelName '/PID_Controller'], 'I', num2str(best_params(2)));
set_param([modelName '/PID_Controller'], 'D', num2str(best_params(3)));

% 运行仿真
simOut = sim(modelName, 'StopTime', '100');

% 提取数据
t_opt = simOut.time_data;
error_opt = simOut.error_data;
output_opt = simOut.output_data;

fprintf('✓ 验证仿真完成\n');

%% 8. 计算并显示性能指标
fprintf('\n性能指标:\n');

% ITAE
itae_opt = trapz(t_opt, t_opt .* abs(error_opt));
fprintf('  ITAE: %.4f\n', itae_opt);

% 超调量
overshoot_opt = (max(output_opt) - 1.0) * 100;
fprintf('  超调量: %.2f%%\n', overshoot_opt);

% 上升时间
idx_10 = find(output_opt >= 0.1, 1, 'first');
idx_90 = find(output_opt >= 0.9, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time_opt = t_opt(idx_90) - t_opt(idx_10);
    fprintf('  上升时间: %.4f 秒\n', rise_time_opt);
end

% 稳态误差
steady_error_opt = abs(mean(error_opt(floor(0.9*length(error_opt)):end)));
fprintf('  稳态误差: %.6f\n', steady_error_opt);

% 调节时间（2%误差带）
settled = find(abs(error_opt) < 0.02);
if ~isempty(settled)
    settling_time_opt = t_opt(settled(end));
    fprintf('  调节时间(2%%): %.4f 秒\n', settling_time_opt);
end

%% 9. 对比优化前后（使用初始参数）
fprintf('\n========================================\n');
fprintf('优化前后对比\n');
fprintf('========================================\n');

% 使用初始参数仿真
set_param([modelName '/PID_Controller'], 'P', '5');
set_param([modelName '/PID_Controller'], 'I', '0.5');
set_param([modelName '/PID_Controller'], 'D', '1');

simOut_init = sim(modelName, 'StopTime', '100');
t_init = simOut_init.time_data;
error_init = simOut_init.error_data;
output_init = simOut_init.output_data;

% 计算初始性能
itae_init = trapz(t_init, t_init .* abs(error_init));
overshoot_init = (max(output_init) - 1.0) * 100;

fprintf('\n初始参数 (Kp=5, Ki=0.5, Kd=1):\n');
fprintf('  ITAE: %.4f\n', itae_init);
fprintf('  超调量: %.2f%%\n', overshoot_init);

fprintf('\n优化后参数 (Kp=%.4f, Ki=%.4f, Kd=%.4f):\n', ...
        best_params(1), best_params(2), best_params(3));
fprintf('  ITAE: %.4f (改善 %.1f%%)\n', itae_opt, (itae_init-itae_opt)/itae_init*100);
fprintf('  超调量: %.2f%% (改善 %.1f%%)\n', overshoot_opt, (overshoot_init-overshoot_opt)/overshoot_init*100);

%% 10. 绘制对比图
fprintf('\n正在绘制对比图...\n');

figure('Position', [100, 100, 1200, 600]);

% 子图1: 输出响应对比
subplot(2, 2, 1);
plot(t_init, ones(size(t_init)), 'k--', 'LineWidth', 1.5); hold on;
plot(t_init, output_init, 'b-', 'LineWidth', 2);
plot(t_opt, output_opt, 'r-', 'LineWidth', 2);
xlabel('时间 (秒)');
ylabel('液位输出 (m)');
title('输出响应对比');
legend('设定值', '优化前', 'GA优化后', 'Location', 'best');
grid on;

% 子图2: 误差对比
subplot(2, 2, 2);
plot(t_init, error_init, 'b-', 'LineWidth', 1.5); hold on;
plot(t_opt, error_opt, 'r-', 'LineWidth', 1.5);
xlabel('时间 (秒)');
ylabel('误差');
title('跟踪误差对比');
legend('优化前', 'GA优化后', 'Location', 'best');
grid on;

% 子图3: 性能指标柱状图
subplot(2, 2, 3);
metrics = [itae_init, itae_opt; overshoot_init, overshoot_opt];
bar(metrics');
set(gca, 'XTickLabel', {'优化前', 'GA优化后'});
ylabel('值');
title('性能指标对比');
legend('ITAE', '超调量(%)', 'Location', 'best');
grid on;

% 子图4: PID参数对比
subplot(2, 2, 4);
params_compare = [5, best_params(1); 0.5, best_params(2); 1, best_params(3)];
bar(params_compare');
set(gca, 'XTickLabel', {'优化前', 'GA优化后'});
ylabel('参数值');
title('PID参数对比');
legend('Kp', 'Ki', 'Kd', 'Location', 'best');
grid on;

% 保存图像
saveas(gcf, 'ga_optimization_results.png');
fprintf('✓ 对比图已保存: ga_optimization_results.png\n');

%% 11. 清理
save_system(modelName);
fprintf('\n✓ 模型已保存\n');

fprintf('\n========================================\n');
fprintf('全部完成！\n');
fprintf('========================================\n');
fprintf('\n生成文件:\n');
fprintf('1. ga_pid_results.mat - 优化结果数据\n');
fprintf('2. ga_optimization_results.png - 对比图\n');
fprintf('========================================\n\n');
