%% run_ga_optimize.m - 遗传算法优化PID参数主程序 (全面优化版)
% 功能：使用遗传算法优化双容水箱PID控制器参数

clear; clc; close all;

fprintf('========================================\n');
fprintf('遗传算法优化PID控制器 (全面优化版)\n');
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

%% 2. 设置GA参数（优化的搜索空间）
fprintf('\n配置遗传算法参数...\n');

n_vars = 3;  % 优化变量个数 [Kp, Ki, Kd]

% 优化的参数搜索边界（基于系统特性缩小范围）
% 对于二阶欠阻尼系统，合理的PID范围应该更保守
%        Kp    Ki    Kd
lb = [  0.5,  0.01,  0.1 ];  % 下限（Kp不能太小）
ub = [ 10.0,  2.0,   5.0 ];  % 上限（Ki不能太大以避免积分饱和）

fprintf('  变量个数: %d\n', n_vars);
fprintf('  Kp范围: [%.2f, %.2f]\n', lb(1), ub(1));
fprintf('  Ki范围: [%.2f, %.2f]\n', lb(2), ub(2));
fprintf('  Kd范围: [%.2f, %.2f]\n', lb(3), ub(3));

%% 3. 配置GA选项（优化的算法参数）
fprintf('\n配置GA选项...\n');

options = optimoptions('ga', ...
    'PopulationSize', 100, ...             % 增大种群，提高多样性
    'MaxGenerations', 100, ...             % 增加代数，充分搜索
    'EliteCount', 10, ...                  % 增加精英数，保留好解
    'CrossoverFraction', 0.85, ...         % 提高交叉率
    'FunctionTolerance', 1e-8, ...         % 更严格的收敛标准
    'MutationFcn', {@mutationadaptfeasible}, ...
    'SelectionFcn', @selectiontournament, ... % 锦标赛选择（比轮盘赌更好）
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf, @gaplotbestindiv, @gaplotrange}, ...
    'UseParallel', false, ...              % 如有并行工具箱可改为true
    'InitialPopulationMatrix', [], ...     % 可选：提供好的初始猜测
    'MaxStallGenerations', 30);            % 30代无改进则停止

% 添加一些好的初始猜测点（基于经验）
initial_guesses = [
    5.0,  0.5,  1.0;   % 原始参数
    3.0,  0.3,  2.0;   % 保守方案
    6.0,  0.6,  1.5;   % 中等方案
    4.0,  0.4,  1.2;   % 平衡方案
    7.0,  0.8,  2.5;   % 激进方案
];
options.InitialPopulationMatrix = initial_guesses;

fprintf('  种群大小: %d\n', options.PopulationSize);
fprintf('  最大代数: %d\n', options.MaxGenerations);
fprintf('  精英保留: %d\n', options.EliteCount);
fprintf('  交叉概率: %.2f\n', options.CrossoverFraction);
fprintf('  初始猜测: %d 组\n', size(initial_guesses, 1));

%% 4. 运行遗传算法
fprintf('\n========================================\n');
fprintf('开始遗传算法优化...\n');
fprintf('========================================\n\n');

tic;  % 开始计时

[best_params, best_cost, exitflag, output_ga] = ga(@calc_fitness_pid, n_vars, ...
                                  [], [], [], [], ...
                                  lb, ub, ...
                                  [], ...
                                  options);

elapsed_time = toc;  % 结束计时

%% 5. 显示优化结果
fprintf('\n========================================\n');
fprintf('优化完成！\n');
fprintf('========================================\n');
fprintf('总耗时: %.2f 秒 (%.2f 分钟)\n', elapsed_time, elapsed_time/60);
fprintf('退出标志: %d\n', exitflag);
fprintf('总代数: %d\n', output_ga.generations);
fprintf('函数评估次数: %d\n', output_ga.funccount);

fprintf('\n最优成本值: %.6f\n\n', best_cost);

fprintf('最优PID参数:\n');
fprintf('  Kp = %.6f\n', best_params(1));
fprintf('  Ki = %.6f\n', best_params(2));
fprintf('  Kd = %.6f\n', best_params(3));

%% 6. 保存优化结果
results_struct = struct(...
    'best_params', best_params, ...
    'best_cost', best_cost, ...
    'elapsed_time', elapsed_time, ...
    'ga_output', output_ga, ...
    'exitflag', exitflag);
save('ga_pid_results.mat', '-struct', 'results_struct');
fprintf('\n✓ 结果已保存到: ga_pid_results.mat\n');

%% 7. 验证仿真（最优参数）
fprintf('\n========================================\n');
fprintf('验证仿真\n');
fprintf('========================================\n');

set_param([modelName '/PID_Controller'], 'P', num2str(best_params(1)));
set_param([modelName '/PID_Controller'], 'I', num2str(best_params(2)));
set_param([modelName '/PID_Controller'], 'D', num2str(best_params(3)));

simOut_opt = sim(modelName, 'StopTime', '100');
t_opt = simOut_opt.time_data;
error_opt = simOut_opt.error_data;
output_opt = simOut_opt.output_data;

fprintf('✓ 验证仿真完成\n');

%% 8. 计算详细性能指标
fprintf('\n性能指标 (优化后):\n');

% ITAE
itae_opt = trapz(t_opt, t_opt .* abs(error_opt));
fprintf('  ITAE: %.4f\n', itae_opt);

% 超调量
overshoot_opt = max(0, (max(output_opt) - 1.0) * 100);
fprintf('  超调量: %.2f%%\n', overshoot_opt);

% 上升时间
idx_10 = find(output_opt >= 0.1, 1, 'first');
idx_90 = find(output_opt >= 0.9, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time_opt = t_opt(idx_90) - t_opt(idx_10);
    fprintf('  上升时间 (10%%-90%%): %.4f 秒\n', rise_time_opt);
end

% 调节时间（2%误差带）
settling_band = 0.02;
unsettled_indices = find(abs(error_opt) >= settling_band);
if ~isempty(unsettled_indices)
    settling_time_opt = t_opt(unsettled_indices(end));
    fprintf('  调节时间 (2%%): %.4f 秒\n', settling_time_opt);
else
    fprintf('  调节时间 (2%%): < %.4f 秒\n', t_opt(end));
end

% 稳态误差
steady_error_opt = abs(mean(error_opt(floor(0.9*length(error_opt)):end)));
fprintf('  稳态误差: %.6f\n', steady_error_opt);

% 后期稳定性
late_start = floor(0.8 * length(output_opt));
late_std = std(output_opt(late_start:end));
fprintf('  后期波动 (标准差): %.6f\n', late_std);

%% 9. 对比优化前后
fprintf('\n========================================\n');
fprintf('优化前后对比\n');
fprintf('========================================\n');

init_P = 5;
init_I = 0.5;
init_D = 1;
set_param([modelName '/PID_Controller'], 'P', num2str(init_P));
set_param([modelName '/PID_Controller'], 'I', num2str(init_I));
set_param([modelName '/PID_Controller'], 'D', num2str(init_D));

simOut_init = sim(modelName, 'StopTime', '100');
t_init = simOut_init.time_data;
error_init = simOut_init.error_data;
output_init = simOut_init.output_data;

% 计算初始性能
itae_init = trapz(t_init, t_init .* abs(error_init));
overshoot_init = max(0, (max(output_init) - 1.0) * 100);
steady_error_init = abs(mean(error_init(floor(0.9*length(error_init)):end)));
late_std_init = std(output_init(floor(0.8*length(output_init)):end));

fprintf('\n初始参数 (Kp=%.1f, Ki=%.1f, Kd=%.1f):\n', init_P, init_I, init_D);
fprintf('  ITAE: %.4f\n', itae_init);
fprintf('  超调量: %.2f%%\n', overshoot_init);
fprintf('  稳态误差: %.6f\n', steady_error_init);
fprintf('  后期波动: %.6f\n', late_std_init);

fprintf('\n优化后参数 (Kp=%.4f, Ki=%.4f, Kd=%.4f):\n', ...
        best_params(1), best_params(2), best_params(3));
fprintf('  ITAE: %.4f (改善 %.1f%%)\n', itae_opt, ...
        max(0, (itae_init-itae_opt)/itae_init*100));
fprintf('  超调量: %.2f%%\n', overshoot_opt);
fprintf('  稳态误差: %.6f (改善 %.1f%%)\n', steady_error_opt, ...
        max(0, (steady_error_init-steady_error_opt)/steady_error_init*100));
fprintf('  后期波动: %.6f (改善 %.1f%%)\n', late_std, ...
        max(0, (late_std_init-late_std)/late_std_init*100));

%% 10. 绘制详细对比图
fprintf('\n正在绘制对比图...\n');

figure('Position', [50, 50, 1400, 800], 'Name', 'GA PID优化结果');

% 子图1: 输出响应对比
subplot(3, 2, 1);
plot(t_init, ones(size(t_init)), 'k--', 'LineWidth', 1.5); hold on;
plot(t_init, output_init, 'b-', 'LineWidth', 2);
plot(t_opt, output_opt, 'r-', 'LineWidth', 2);
xlabel('时间 (秒)', 'FontSize', 11);
ylabel('液位输出 (m)', 'FontSize', 11);
title('输出响应对比', 'FontSize', 12, 'FontWeight', 'bold');
legend('设定值', sprintf('优化前 (P=%.1f, I=%.1f, D=%.1f)', init_P, init_I, init_D), ...
       sprintf('GA优化后 (P=%.2f, I=%.2f, D=%.2f)', best_params), ...
       'Location', 'southeast');
grid on;
ylim([0, 1.3]);
xlim([0, 100]);

% 子图2: 误差对比
subplot(3, 2, 2);
plot(t_init, error_init, 'b-', 'LineWidth', 1.5); hold on;
plot(t_opt, error_opt, 'r-', 'LineWidth', 1.5);
plot([0, 100], [0.02, 0.02], 'g--', 'LineWidth', 1);
plot([0, 100], [-0.02, -0.02], 'g--', 'LineWidth', 1);
xlabel('时间 (秒)', 'FontSize', 11);
ylabel('误差', 'FontSize', 11);
title('跟踪误差对比', 'FontSize', 12, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', '±2%误差带', 'Location', 'best');
grid on;
ylim([-0.2, 1.2]);
xlim([0, 100]);

% 子图3: 后期放大图（检查稳定性）
subplot(3, 2, 3);
late_start_idx = find(t_opt >= 80, 1);
plot(t_init(late_start_idx:end), output_init(late_start_idx:end), 'b-', 'LineWidth', 2); hold on;
plot(t_opt(late_start_idx:end), output_opt(late_start_idx:end), 'r-', 'LineWidth', 2);
plot([80, 100], [1, 1], 'k--', 'LineWidth', 1.5);
xlabel('时间 (秒)', 'FontSize', 11);
ylabel('液位输出 (m)', 'FontSize', 11);
title('后期稳定性对比 (t > 80s)', 'FontSize', 12, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', '设定值', 'Location', 'best');
grid on;
ylim([0.95, 1.05]);

% 子图4: 性能指标雷达图
subplot(3, 2, 4);
% 归一化性能指标用于对比
metrics_normalized = [
    itae_init/max(itae_init, itae_opt);
    overshoot_init/max(overshoot_init, overshoot_opt+0.01);
    steady_error_init/max(steady_error_init, steady_error_opt);
    late_std_init/max(late_std_init, late_std);
    itae_opt/max(itae_init, itae_opt);
    overshoot_opt/max(overshoot_init, overshoot_opt+0.01);
    steady_error_opt/max(steady_error_init, steady_error_opt);
    late_std/max(late_std_init, late_std);
];
categories = {'ITAE', '超调量', '稳态误差', '后期波动'};
bar_data = reshape(metrics_normalized, 4, 2)';
b = bar(bar_data);
set(gca, 'XTickLabel', {'优化前', 'GA优化后'});
ylabel('归一化值 (越小越好)', 'FontSize', 11);
title('性能指标对比', 'FontSize', 12, 'FontWeight', 'bold');
legend(categories, 'Location', 'best');
grid on;

% 子图5: PID参数对比
subplot(3, 2, 5);
params_data = [init_P, best_params(1); init_I, best_params(2); init_D, best_params(3)];
b = bar(params_data);
set(gca, 'XTickLabel', {'Kp', 'Ki', 'Kd'});
ylabel('参数值', 'FontSize', 11);
title('PID参数对比', 'FontSize', 12, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', 'Location', 'best');
grid on;

% 子图6: GA收敛曲线
subplot(3, 2, 6);
if isfield(output_ga, 'bestfval')
    plot(1:length(output_ga.bestfval), output_ga.bestfval, 'b-', 'LineWidth', 2);
    xlabel('代数', 'FontSize', 11);
    ylabel('最优成本值', 'FontSize', 11);
    title('GA收敛过程', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
else
    text(0.5, 0.5, '收敛数据不可用', 'HorizontalAlignment', 'center');
end

% 保存图像
saveas(gcf, 'ga_optimization_results.png');
fprintf('✓ 对比图已保存: ga_optimization_results.png\n');

%% 11. 生成性能报告
fprintf('\n========================================\n');
fprintf('性能报告\n');
fprintf('========================================\n');
fprintf('系统稳定性: %s\n', ...
    iif(late_std < 0.01, '优秀 ✓', iif(late_std < 0.05, '良好 ✓', '需改进 ✗')));
fprintf('超调控制: %s\n', ...
    iif(overshoot_opt < 5, '优秀 ✓', iif(overshoot_opt < 10, '良好 ✓', '需改进 ✗')));
fprintf('稳态精度: %s\n', ...
    iif(steady_error_opt < 0.01, '优秀 ✓', iif(steady_error_opt < 0.05, '良好 ✓', '需改进 ✗')));

%% 12. 清理
save_system(modelName);
fprintf('\n✓ 模型已保存\n');

fprintf('\n========================================\n');
fprintf('全部完成！\n');
fprintf('========================================\n');
fprintf('\n生成文件:\n');
fprintf('1. ga_pid_results.mat - 优化结果数据\n');
fprintf('2. ga_optimization_results.png - 详细对比图\n');
fprintf('========================================\n\n');

% 辅助函数
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end