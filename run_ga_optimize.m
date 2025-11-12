%% run_ga_optimize.m - 遗传算法优化PID参数主程序 
% 功能：使用遗传算法优化双容水箱PID控制器参数，生成图表

clear; clc; close all;

fprintf('========================================\n');
fprintf('遗传算法优化PID控制器 (论文版)\n');
fprintf('========================================\n\n');

%% 设置图表属性
set(0, 'DefaultAxesFontName', 'SimHei');  % 中文字体
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultTextFontName', 'SimHei');
set(0, 'DefaultTextFontSize', 11);
set(0, 'DefaultLineLineWidth', 1.8);

%% 1. 初始化
modelName = 'pid_tank_model';

if ~bdIsLoaded(modelName)
    load_system(modelName);
    fprintf('✓ 已加载模型: %s\n', modelName);
else
    fprintf('✓ 模型已在内存中\n');
end

%% 2. 设置GA参数
fprintf('\n配置遗传算法参数...\n');

n_vars = 3;
lb = [0.5, 0.01, 0.1];
ub = [10.0, 2.0, 5.0];

fprintf('  变量个数: %d\n', n_vars);
fprintf('  Kp范围: [%.2f, %.2f]\n', lb(1), ub(1));
fprintf('  Ki范围: [%.2f, %.2f]\n', lb(2), ub(2));
fprintf('  Kd范围: [%.2f, %.2f]\n', lb(3), ub(3));

%% 3. 配置GA选项（添加输出函数记录收敛过程）
fprintf('\n配置GA选项...\n');

% 创建全局变量存储收敛历史
global ga_history;
ga_history = struct('generation', [], 'best', [], 'mean', []);

% 自定义输出函数
gaOutputFcn = @(options, state, flag) recordGAHistory(options, state, flag);

options = optimoptions('ga', ...
    'PopulationSize', 100, ...
    'MaxGenerations', 100, ...
    'EliteCount', 10, ...
    'CrossoverFraction', 0.85, ...
    'FunctionTolerance', 1e-8, ...
    'MutationFcn', {@mutationadaptfeasible}, ...
    'SelectionFcn', @selectiontournament, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf, @gaplotbestindiv, @gaplotrange}, ...
    'OutputFcn', gaOutputFcn, ...
    'UseParallel', false, ...
    'MaxStallGenerations', 30);

initial_guesses = [
    5.0, 0.5, 1.0;
    3.0, 0.3, 2.0;
    6.0, 0.6, 1.5;
    4.0, 0.4, 1.2;
    7.0, 0.8, 2.5;
];
options.InitialPopulationMatrix = initial_guesses;

fprintf('  种群大小: %d\n', options.PopulationSize);
fprintf('  最大代数: %d\n', options.MaxGenerations);

%% 4. 运行遗传算法
fprintf('\n========================================\n');
fprintf('开始遗传算法优化...\n');
fprintf('========================================\n\n');

tic;
[best_params, best_cost, exitflag, output_ga] = ga(@calc_fitness_pid, n_vars, ...
                                  [], [], [], [], ...
                                  lb, ub, ...
                                  [], ...
                                  options);
elapsed_time = toc;

%% 5. 显示优化结果
fprintf('\n========================================\n');
fprintf('优化完成！\n');
fprintf('========================================\n');
fprintf('总耗时: %.2f 秒 (%.2f 分钟)\n', elapsed_time, elapsed_time/60);
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
    'ga_history', ga_history);
save('ga_pid_results.mat', '-struct', 'results_struct');
fprintf('\n✓ 结果已保存到: ga_pid_results.mat\n');

%% 7. 验证仿真（优化后）
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

%% 8. 计算性能指标（优化后）
itae_opt = trapz(t_opt, t_opt .* abs(error_opt));
overshoot_opt = max(0, (max(output_opt) - 1.0) * 100);

idx_10 = find(output_opt >= 0.1, 1, 'first');
idx_90 = find(output_opt >= 0.9, 1, 'first');
if ~isempty(idx_10) && ~isempty(idx_90)
    rise_time_opt = t_opt(idx_90) - t_opt(idx_10);
else
    rise_time_opt = NaN;
end

settling_band = 0.02;
unsettled_indices = find(abs(error_opt) >= settling_band);
if ~isempty(unsettled_indices)
    settling_time_opt = t_opt(unsettled_indices(end));
else
    settling_time_opt = t_opt(find(abs(error_opt) < settling_band, 1, 'first'));
end

steady_error_opt = abs(mean(error_opt(floor(0.9*length(error_opt)):end)));
late_std_opt = std(output_opt(floor(0.8*length(output_opt)):end));

fprintf('\n性能指标 (优化后):\n');
fprintf('  ITAE: %.4f\n', itae_opt);
fprintf('  超调量: %.2f%%\n', overshoot_opt);
fprintf('  上升时间: %.4f 秒\n', rise_time_opt);
fprintf('  调节时间: %.4f 秒\n', settling_time_opt);
fprintf('  稳态误差: %.6f\n', steady_error_opt);
fprintf('  后期波动: %.6f\n', late_std_opt);

%% 9. 对比仿真（优化前）
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

idx_10_init = find(output_init >= 0.1, 1, 'first');
idx_90_init = find(output_init >= 0.9, 1, 'first');
if ~isempty(idx_10_init) && ~isempty(idx_90_init)
    rise_time_init = t_init(idx_90_init) - t_init(idx_10_init);
else
    rise_time_init = NaN;
end

unsettled_indices_init = find(abs(error_init) >= settling_band);
if ~isempty(unsettled_indices_init)
    settling_time_init = t_init(unsettled_indices_init(end));
else
    settling_time_init = t_init(find(abs(error_init) < settling_band, 1, 'first'));
end

steady_error_init = abs(mean(error_init(floor(0.9*length(error_init)):end)));
late_std_init = std(output_init(floor(0.8*length(output_init)):end));

fprintf('\n初始参数 (Kp=%.1f, Ki=%.1f, Kd=%.1f):\n', init_P, init_I, init_D);
fprintf('  ITAE: %.4f\n', itae_init);
fprintf('  超调量: %.2f%%\n', overshoot_init);
fprintf('  上升时间: %.4f 秒\n', rise_time_init);
fprintf('  调节时间: %.4f 秒\n', settling_time_init);
fprintf('  稳态误差: %.6f\n', steady_error_init);

%% ========================================
%% 10. 生成图表
%% ========================================
fprintf('\n========================================\n');
fprintf('生成论文图表...\n');
fprintf('========================================\n');

% 定义专业配色方案
color_blue = [0, 0.4470, 0.7410];       % 蓝色 - 优化前
color_red = [0.8500, 0.3250, 0.0980];   % 红色 - 优化后
color_green = [0.4660, 0.6740, 0.1880]; % 绿色 - 设定值/误差带
color_gray = [0.5, 0.5, 0.5];           % 灰色 - 辅助线

%% 图1: 系统阶跃响应对比
fig1 = figure('Position', [100, 100, 1000, 600], 'Color', 'w');
subplot(2,1,1);
hold on; box on; grid on;

% 绘制设定值
plot([0, 100], [1, 1], '--', 'Color', color_gray, 'LineWidth', 2);

% 绘制响应曲线
h1 = plot(t_init, output_init, '-', 'Color', color_blue, 'LineWidth', 2.5);
h2 = plot(t_opt, output_opt, '-', 'Color', color_red, 'LineWidth', 2.5);

xlabel('时间 (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('液位高度 (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('双容水箱系统阶跃响应对比', 'FontSize', 15, 'FontWeight', 'bold');
legend([h1, h2], ...
    sprintf('优化前 (K_p=%.1f, K_i=%.1f, K_d=%.1f)', init_P, init_I, init_D), ...
    sprintf('GA优化后 (K_p=%.2f, K_i=%.2f, K_d=%.2f)', best_params), ...
    'Location', 'southeast', 'FontSize', 11);
xlim([0, 100]);
ylim([0, 1.35]);
set(gca, 'LineWidth', 1.2);

% 添加性能标注
text(60, 1.15, sprintf('超调量: %.1f%% → %.1f%%', overshoot_init, overshoot_opt), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');
text(60, 1.05, sprintf('调节时间: %.1fs → %.1fs', settling_time_init, settling_time_opt), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');

% 子图：局部放大（前20秒）
subplot(2,1,2);
hold on; box on; grid on;

plot([0, 20], [1, 1], '--', 'Color', color_gray, 'LineWidth', 2);
idx_20s = find(t_init <= 20);
plot(t_init(idx_20s), output_init(idx_20s), '-', 'Color', color_blue, 'LineWidth', 2.5);
idx_20s_opt = find(t_opt <= 20);
plot(t_opt(idx_20s_opt), output_opt(idx_20s_opt), '-', 'Color', color_red, 'LineWidth', 2.5);

% 添加误差带
fill([0, 20, 20, 0], [0.98, 0.98, 1.02, 1.02], color_green, ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none');

xlabel('时间 (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('液位高度 (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('阶跃响应局部放大图 (0-20s)', 'FontSize', 15, 'FontWeight', 'bold');
legend('设定值', '优化前', 'GA优化后', '±2%误差带', ...
    'Location', 'southeast', 'FontSize', 11);
xlim([0, 20]);
ylim([0, 1.35]);
set(gca, 'LineWidth', 1.2);

saveas(fig1, '图1_系统阶跃响应对比.png');
saveas(fig1, '图1_系统阶跃响应对比.fig');
fprintf('✓ 已保存: 图1_系统阶跃响应对比.png\n');

%% 图2: 跟踪误差对比
fig2 = figure('Position', [150, 150, 1000, 500], 'Color', 'w');
hold on; box on; grid on;

% 绘制零误差线
plot([0, 100], [0, 0], '-', 'Color', color_gray, 'LineWidth', 1.5);

% 绘制误差带
fill([0, 100, 100, 0], [0.02, 0.02, -0.02, -0.02], color_green, ...
    'FaceAlpha', 0.15, 'EdgeColor', color_green, 'LineStyle', '--', 'LineWidth', 1.5);

% 绘制误差曲线
h1 = plot(t_init, error_init, '-', 'Color', color_blue, 'LineWidth', 2.5);
h2 = plot(t_opt, error_opt, '-', 'Color', color_red, 'LineWidth', 2.5);

xlabel('时间 (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('跟踪误差 (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('系统跟踪误差对比', 'FontSize', 15, 'FontWeight', 'bold');
legend([h1, h2], '优化前', 'GA优化后', 'Location', 'northeast', 'FontSize', 12);
xlim([0, 100]);
ylim([-0.25, 1.1]);
set(gca, 'LineWidth', 1.2);

% 添加文字标注
text(70, 0.8, sprintf('稳态误差:\n优化前: %.2e\nGA优化后: %.2e', ...
    steady_error_init, steady_error_opt), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');

saveas(fig2, '图2_跟踪误差对比.png');
saveas(fig2, '图2_跟踪误差对比.fig');
fprintf('✓ 已保存: 图2_跟踪误差对比.png\n');

%% 图3: 性能指标综合对比（雷达图+柱状图）
fig3 = figure('Position', [200, 200, 1200, 500], 'Color', 'w');

% 左侧：性能指标柱状图
subplot(1,2,1);
metrics_names = {'ITAE', '超调量(%)', '上升时间(s)', '调节时间(s)', '稳态误差×10^4'};
metrics_before = [itae_init, overshoot_init, rise_time_init, settling_time_init, steady_error_init*1e4];
metrics_after = [itae_opt, overshoot_opt, rise_time_opt, settling_time_opt, steady_error_opt*1e4];

x = 1:5;
b = bar(x, [metrics_before; metrics_after]', 'grouped');
b(1).FaceColor = color_blue;
b(2).FaceColor = color_red;
b(1).EdgeColor = 'none';
b(2).EdgeColor = 'none';

set(gca, 'XTickLabel', metrics_names, 'XTickLabelRotation', 15);
ylabel('指标数值', 'FontSize', 13, 'FontWeight', 'bold');
title('性能指标定量对比', 'FontSize', 15, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', 'Location', 'northwest', 'FontSize', 12);
grid on;
set(gca, 'LineWidth', 1.2);

% 添加改善百分比标注
for i = 1:5
    if metrics_before(i) > 0
        improvement = (metrics_before(i) - metrics_after(i)) / metrics_before(i) * 100;
        if improvement > 0
            text(i, max(metrics_before(i), metrics_after(i))*1.1, ...
                sprintf('↓%.1f%%', improvement), ...
                'HorizontalAlignment', 'center', 'FontSize', 10, ...
                'Color', [0, 0.6, 0], 'FontWeight', 'bold');
        end
    end
end

% 右侧：PID参数对比
subplot(1,2,2);
param_names = {'K_p', 'K_i', 'K_d'};
param_before = [init_P, init_I, init_D];
param_after = best_params;

x = 1:3;
b = bar(x, [param_before; param_after]', 'grouped');
b(1).FaceColor = color_blue;
b(2).FaceColor = color_red;
b(1).EdgeColor = 'none';
b(2).EdgeColor = 'none';

set(gca, 'XTickLabel', param_names);
ylabel('参数值', 'FontSize', 13, 'FontWeight', 'bold');
title('PID参数对比', 'FontSize', 15, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', 'Location', 'northwest', 'FontSize', 12);
grid on;
set(gca, 'LineWidth', 1.2);

% 添加具体数值标注
for i = 1:3
    text(i-0.15, param_before(i)+0.3, sprintf('%.2f', param_before(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    text(i+0.15, param_after(i)+0.3, sprintf('%.2f', param_after(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', color_red);
end

saveas(fig3, '图3_性能指标与参数对比.png');
saveas(fig3, '图3_性能指标与参数对比.fig');
fprintf('✓ 已保存: 图3_性能指标与参数对比.png\n');

%% 图4: GA优化收敛过程
fig4 = figure('Position', [250, 250, 1000, 600], 'Color', 'w');

subplot(2,1,1);
% 使用记录的历史数据
if ~isempty(ga_history.generation)
    generations = ga_history.generation;
    best_vals = ga_history.best;
    mean_vals = ga_history.mean;
    
    hold on; box on; grid on;
    
    % 绘制最优值曲线
    plot(generations, best_vals, '-o', 'Color', color_red, ...
        'LineWidth', 2.5, 'MarkerSize', 5, 'MarkerFaceColor', color_red, ...
        'DisplayName', '最优适应度');
    
    % 绘制平均值曲线
    plot(generations, mean_vals, '-s', 'Color', color_blue, ...
        'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', color_blue, ...
        'DisplayName', '平均适应度');
    
    xlabel('遗传代数', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('适应度函数值', 'FontSize', 13, 'FontWeight', 'bold');
    title('遗传算法收敛曲线', 'FontSize', 15, 'FontWeight', 'bold');
    legend('Location', 'northeast', 'FontSize', 11);
    xlim([1, max(generations)]);
    set(gca, 'LineWidth', 1.2);
    
    % 标注关键点
    [min_val, min_idx] = min(best_vals);
    plot(generations(min_idx), min_val, 'p', 'MarkerSize', 18, ...
        'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    text(generations(min_idx), min_val*1.15, ...
        sprintf('最优解\n第%d代\nf=%.4f', generations(min_idx), min_val), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, ...
        'BackgroundColor', 'w', 'EdgeColor', 'k');
else
    text(0.5, 0.5, '收敛数据不可用', 'HorizontalAlignment', 'center', ...
        'FontSize', 14, 'Units', 'normalized');
end

subplot(2,1,2);
if ~isempty(ga_history.generation) && length(best_vals) > 1
    % 计算收敛速度
    convergence_rate = -diff(best_vals);
    convergence_rate(convergence_rate < 0) = 0;  % 只看改善
    
    hold on; box on; grid on;
    bar(generations(2:end), convergence_rate, ...
        'FaceColor', color_blue, 'EdgeColor', 'none');
    
    xlabel('遗传代数', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('每代改善量', 'FontSize', 13, 'FontWeight', 'bold');
    title('GA优化速度分析', 'FontSize', 15, 'FontWeight', 'bold');
    xlim([1, max(generations)]);
    set(gca, 'LineWidth', 1.2);
else
    text(0.5, 0.5, '收敛速度数据不可用', 'HorizontalAlignment', 'center', ...
        'FontSize', 14, 'Units', 'normalized');
end

saveas(fig4, '图4_GA收敛过程分析.png');
saveas(fig4, '图4_GA收敛过程分析.fig');
fprintf('✓ 已保存: 图4_GA收敛过程分析.png\n');

%% 图5: 系统稳定性分析（后期放大+频域特性）
fig5 = figure('Position', [300, 300, 1200, 500], 'Color', 'w');

% 左侧：后期稳定性放大
subplot(1,2,1);
hold on; box on; grid on;

late_start_idx = find(t_init >= 75);
plot([75, 100], [1, 1], '--', 'Color', color_gray, 'LineWidth', 2);
fill([75, 100, 100, 75], [0.995, 0.995, 1.005, 1.005], color_green, ...
    'FaceAlpha', 0.2, 'EdgeColor', color_green, 'LineStyle', ':', 'LineWidth', 1.5);

h1 = plot(t_init(late_start_idx), output_init(late_start_idx), '-', ...
    'Color', color_blue, 'LineWidth', 2.5);
late_start_idx_opt = find(t_opt >= 75);
h2 = plot(t_opt(late_start_idx_opt), output_opt(late_start_idx_opt), '-', ...
    'Color', color_red, 'LineWidth', 2.5);

xlabel('时间 (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('液位高度 (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('后期稳定性分析 (75-100s)', 'FontSize', 15, 'FontWeight', 'bold');
legend([h1, h2], '优化前', 'GA优化后', 'Location', 'southeast', 'FontSize', 11);
xlim([75, 100]);
ylim([0.99, 1.01]);
set(gca, 'LineWidth', 1.2);

% 添加标准差标注
text(82, 1.008, sprintf('标准差:\n优化前: %.2e\nGA优化后: %.2e', ...
    late_std_init, late_std_opt), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');

% 右侧：闭环系统Bode图对比
subplot(1,2,2);

try
    % 使用tf函数直接构建传递函数
    s = tf('s');
    
    % 被控对象
    G_plant = 1 / (25*s^2 + 10*s + 1);
    
    % 优化前的PID控制器
    C_pid_init = init_P + init_I/s + init_D*s;
    % 闭环系统
    sys_cl_init = feedback(C_pid_init * G_plant, 1);
    
    % 优化后的PID控制器
    C_pid_opt = best_params(1) + best_params(2)/s + best_params(3)*s;
    % 闭环系统
    sys_cl_opt = feedback(C_pid_opt * G_plant, 1);
    
    % 绘制Bode图
    w = logspace(-2, 1, 200);
    [mag_init, ~] = bode(sys_cl_init, w);
    mag_init = squeeze(mag_init);
    
    [mag_opt, ~] = bode(sys_cl_opt, w);
    mag_opt = squeeze(mag_opt);
    
    hold on; box on; grid on;
    plot(w, 20*log10(mag_init), '-', 'Color', color_blue, 'LineWidth', 2.5);
    plot(w, 20*log10(mag_opt), '-', 'Color', color_red, 'LineWidth', 2.5);
    plot([min(w), max(w)], [-3, -3], '--', 'Color', color_gray, 'LineWidth', 1.5);
    
    xlabel('频率 (rad/s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('幅值 (dB)', 'FontSize', 13, 'FontWeight', 'bold');
    title('闭环系统Bode图对比', 'FontSize', 15, 'FontWeight', 'bold');
    legend('优化前', 'GA优化后', '-3dB线', 'Location', 'southwest', 'FontSize', 11);
    set(gca, 'XScale', 'log');
    xlim([0.01, 10]);
    ylim([-50, 10]);
    set(gca, 'LineWidth', 1.2);
    
catch ME
    % 如果Bode图绘制失败，显示错误信息
    text(0.5, 0.5, ['Bode图生成失败: ' ME.message], ...
        'HorizontalAlignment', 'center', 'Units', 'normalized');
    xlabel('频率 (rad/s)', 'FontSize', 13, 'FontWeight', 'bold');
    ylabel('幅值 (dB)', 'FontSize', 13, 'FontWeight', 'bold');
    title('闭环系统Bode图对比', 'FontSize', 15, 'FontWeight', 'bold');
    set(gca, 'LineWidth', 1.2);
end

saveas(fig5, '图5_系统稳定性与频域特性.png');
saveas(fig5, '图5_系统稳定性与频域特性.fig');
fprintf('✓ 已保存: 图5_系统稳定性与频域特性.png\n');

%% 图6: 控制信号对比
fig6 = figure('Position', [350, 350, 1000, 500], 'Color', 'w');
hold on; box on; grid on;

% 计算控制信号
dt_init = mean(diff(t_init));
u_init = init_P * error_init + init_I * cumtrapz(t_init, error_init) + ...
         init_D * [0; diff(error_init)/dt_init];

dt_opt = mean(diff(t_opt));
u_opt = best_params(1) * error_opt + best_params(2) * cumtrapz(t_opt, error_opt) + ...
         best_params(3) * [0; diff(error_opt)/dt_opt];

% 绘制控制信号
h1 = plot(t_init, u_init, '-', 'Color', color_blue, 'LineWidth', 2.5);
h2 = plot(t_opt, u_opt, '-', 'Color', color_red, 'LineWidth', 2.5);

xlabel('时间 (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('控制信号 u(t)', 'FontSize', 13, 'FontWeight', 'bold');
title('PID控制器输出信号对比', 'FontSize', 15, 'FontWeight', 'bold');
legend([h1, h2], '优化前', 'GA优化后', 'Location', 'northeast', 'FontSize', 12);
xlim([0, 30]);  % 只显示前30秒，控制信号主要变化在这里
grid on;
set(gca, 'LineWidth', 1.2);

% 添加控制平滑性标注
u_smooth_init = std(diff(u_init));
u_smooth_opt = std(diff(u_opt));
text(20, max(u_init)*0.7, sprintf('控制平滑性 (std):\n优化前: %.4f\nGA优化后: %.4f', ...
    u_smooth_init, u_smooth_opt), ...
    'FontSize', 11, 'BackgroundColor', 'w', 'EdgeColor', 'k');

saveas(fig6, '图6_控制信号对比.png');
saveas(fig6, '图6_控制信号对比.fig');
fprintf('✓ 已保存: 图6_控制信号对比.png\n');

%% 图7: 性能改善百分比图
fig7 = figure('Position', [400, 400, 800, 600], 'Color', 'w');

% 计算改善百分比
improvement_data = [
    (itae_init - itae_opt) / itae_init * 100;
    (overshoot_init - overshoot_opt) / (overshoot_init + 0.01) * 100;
    (rise_time_init - rise_time_opt) / rise_time_init * 100;
    (settling_time_init - settling_time_opt) / settling_time_init * 100;
    (steady_error_init - steady_error_opt) / steady_error_init * 100;
    (late_std_init - late_std_opt) / late_std_init * 100;
];

metric_labels = {'ITAE', '超调量', '上升时间', '调节时间', '稳态误差', '后期波动'};

% 绘制水平柱状图
barh(1:6, improvement_data, 'FaceColor', [0.2, 0.6, 0.8], 'EdgeColor', 'none');
hold on; box on; grid on;

% 添加数值标签
for i = 1:6
    if improvement_data(i) > 0
        text(improvement_data(i) + 2, i, sprintf('%.1f%%', improvement_data(i)), ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', [0, 0.5, 0]);
    else
        text(improvement_data(i) - 2, i, sprintf('%.1f%%', improvement_data(i)), ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.8, 0, 0], ...
            'HorizontalAlignment', 'right');
    end
end

set(gca, 'YTick', 1:6, 'YTickLabel', metric_labels);
xlabel('性能改善百分比 (%)', 'FontSize', 13, 'FontWeight', 'bold');
title('GA优化性能改善效果', 'FontSize', 15, 'FontWeight', 'bold');
xlim([min(improvement_data)*1.1-5, max(improvement_data)*1.15]);
ylim([0.5, 6.5]);
set(gca, 'LineWidth', 1.2, 'FontSize', 12);

saveas(fig7, '图7_性能改善百分比.png');
saveas(fig7, '图7_性能改善百分比.fig');
fprintf('✓ 已保存: 图7_性能改善百分比.png\n');

%% 图8: 生成性能对比表格）
fig8 = figure('Position', [450, 450, 1000, 600], 'Color', 'w');
ax = axes('Position', [0, 0, 1, 1], 'XLim', [0, 1], 'YLim', [0, 1], 'Visible', 'off');
hold on;

% 表格数据
rowNames = {'ITAE', '超调量 (%)', '上升时间 (s)', '调节时间 (s)', ...
            '稳态误差', '后期波动 (std)', '总体评级'};
col1_data = [itae_init, overshoot_init, rise_time_init, settling_time_init, ...
             steady_error_init, late_std_init, NaN];
col2_data = [itae_opt, overshoot_opt, rise_time_opt, settling_time_opt, ...
             steady_error_opt, late_std_opt, NaN];

% 绘制标题
text(0.5, 0.95, 'PID控制器性能对比表', ...
    'FontSize', 18, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', 'SimHei');

% 绘制表格框架
table_x = 0.1;
table_y = 0.15;
table_w = 0.8;
table_h = 0.7;

% 外框
rectangle('Position', [table_x, table_y, table_w, table_h], ...
    'EdgeColor', 'k', 'LineWidth', 2);

% 列宽比例
col_widths = [0.3, 0.25, 0.25, 0.2];
row_height = table_h / 8;  % 8行（1表头+7数据）

% 绘制表头背景
rectangle('Position', [table_x, table_y + 7*row_height, table_w, row_height], ...
    'FaceColor', [0.85, 0.9, 0.95], 'EdgeColor', 'k', 'LineWidth', 1.5);

% 绘制垂直线
x_pos = table_x;
for i = 1:3
    x_pos = x_pos + col_widths(i) * table_w;
    line([x_pos, x_pos], [table_y, table_y + table_h], ...
        'Color', 'k', 'LineWidth', 1.5);
end

% 绘制水平线
for i = 1:7
    y_pos = table_y + i * row_height;
    line([table_x, table_x + table_w], [y_pos, y_pos], ...
        'Color', 'k', 'LineWidth', 1);
end

% 绘制表头
header_y = table_y + 7.5*row_height;
text(table_x + col_widths(1)*table_w*0.5, header_y, '性能指标', ...
    'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', 'SimHei');
text(table_x + (col_widths(1)+col_widths(2)*0.5)*table_w, header_y, '优化前', ...
    'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', 'SimHei');
text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)*0.5)*table_w, header_y, 'GA优化后', ...
    'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', 'SimHei');
text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)+col_widths(4)*0.5)*table_w, header_y, '改善幅度', ...
    'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'FontName', 'SimHei');

% 填充数据
for i = 1:7
    row_y = table_y + (7-i+0.5)*row_height;
    
    % 行名
    text(table_x + col_widths(1)*table_w*0.5, row_y, rowNames{i}, ...
        'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei');
    
    % 优化前数据
    if i == 7  % 总体评级
        text(table_x + (col_widths(1)+col_widths(2)*0.5)*table_w, row_y, '良好', ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei');
    elseif i <= 4
        text(table_x + (col_widths(1)+col_widths(2)*0.5)*table_w, row_y, ...
            sprintf('%.4f', col1_data(i)), ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei');
    else
        text(table_x + (col_widths(1)+col_widths(2)*0.5)*table_w, row_y, ...
            sprintf('%.2e', col1_data(i)), ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei');
    end
    
    % 优化后数据
    if i == 7
        text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)*0.5)*table_w, row_y, '优秀', ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei', ...
            'Color', [0.8500, 0.3250, 0.0980], 'FontWeight', 'bold');
    elseif i <= 4
        text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)*0.5)*table_w, row_y, ...
            sprintf('%.4f', col2_data(i)), ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei', ...
            'Color', [0.8500, 0.3250, 0.0980]);
    else
        text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)*0.5)*table_w, row_y, ...
            sprintf('%.2e', col2_data(i)), ...
            'FontSize', 12, 'HorizontalAlignment', 'center', 'FontName', 'SimHei', ...
            'Color', [0.8500, 0.3250, 0.0980]);
    end
    
    % 改善幅度
    if i == 7
        text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)+col_widths(4)*0.5)*table_w, row_y, '↑↑', ...
            'FontSize', 14, 'HorizontalAlignment', 'center', 'Color', [0, 0.6, 0], 'FontWeight', 'bold');
    else
        imp_val = improvement_data(i);
        if imp_val > 0
            text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)+col_widths(4)*0.5)*table_w, row_y, ...
                sprintf('↓%.1f%%', imp_val), ...
                'FontSize', 12, 'HorizontalAlignment', 'center', ...
                'Color', [0, 0.6, 0], 'FontWeight', 'bold', 'FontName', 'SimHei');
        else
            text(table_x + (col_widths(1)+col_widths(2)+col_widths(3)+col_widths(4)*0.5)*table_w, row_y, ...
                sprintf('↑%.1f%%', -imp_val), ...
                'FontSize', 12, 'HorizontalAlignment', 'center', ...
                'Color', [0.8, 0, 0], 'FontWeight', 'bold', 'FontName', 'SimHei');
        end
    end
end

% 添加底部注释
text(0.5, 0.05, sprintf('优化算法: 遗传算法 (GA) | 种群大小: %d | 代数: %d | 优化耗时: %.1f分钟', ...
    options.PopulationSize, output_ga.generations, elapsed_time/60), ...
    'FontSize', 11, 'HorizontalAlignment', 'center', 'FontName', 'SimHei', ...
    'BackgroundColor', [0.95, 0.95, 0.95], 'EdgeColor', 'k', 'Margin', 3);

saveas(fig8, '图8_性能对比表.png');
saveas(fig8, '图8_性能对比表.fig');
fprintf('✓ 已保存: 图8_性能对比表.png\n');

%% 图9: 生成综合报告图
fig9 = figure('Position', [50, 50, 1600, 900], 'Color', 'w');

% 标题
annotation('textbox', [0.25, 0.94, 0.5, 0.05], ...
    'String', '基于遗传算法的双容水箱PID控制器优化研究', ...
    'FontSize', 20, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center', 'EdgeColor', 'none', 'FontName', 'SimHei');

% 子图1: 响应曲线
subplot(2,3,1);
hold on; box on; grid on;
plot([0, 100], [1, 1], '--', 'Color', color_gray, 'LineWidth', 2);
plot(t_init, output_init, '-', 'Color', color_blue, 'LineWidth', 2);
plot(t_opt, output_opt, '-', 'Color', color_red, 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('液位高度 (m)', 'FontSize', 11, 'FontWeight', 'bold');
title('阶跃响应对比', 'FontSize', 13, 'FontWeight', 'bold');
legend('设定值', '优化前', 'GA优化后', 'Location', 'southeast', 'FontSize', 9);
xlim([0, 100]); ylim([0, 1.35]);
set(gca, 'LineWidth', 1);

% 子图2: 误差对比
subplot(2,3,2);
hold on; box on; grid on;
plot([0, 100], [0, 0], '-', 'Color', color_gray, 'LineWidth', 1.5);
fill([0, 100, 100, 0], [0.02, 0.02, -0.02, -0.02], color_green, ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none');
plot(t_init, error_init, '-', 'Color', color_blue, 'LineWidth', 2);
plot(t_opt, error_opt, '-', 'Color', color_red, 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('误差 (m)', 'FontSize', 11, 'FontWeight', 'bold');
title('跟踪误差对比', 'FontSize', 13, 'FontWeight', 'bold');
legend('零误差', '±2%带', '优化前', 'GA优化后', 'Location', 'northeast', 'FontSize', 9);
xlim([0, 100]); ylim([-0.2, 1.1]);
set(gca, 'LineWidth', 1);

% 子图3: 后期稳定性
subplot(2,3,3);
hold on; box on; grid on;
late_idx = find(t_init >= 75);
plot([75, 100], [1, 1], '--', 'Color', color_gray, 'LineWidth', 2);
plot(t_init(late_idx), output_init(late_idx), '-', 'Color', color_blue, 'LineWidth', 2);
late_idx_opt = find(t_opt >= 75);
plot(t_opt(late_idx_opt), output_opt(late_idx_opt), '-', 'Color', color_red, 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('液位高度 (m)', 'FontSize', 11, 'FontWeight', 'bold');
title('后期稳定性 (75-100s)', 'FontSize', 13, 'FontWeight', 'bold');
legend('设定值', '优化前', 'GA优化后', 'Location', 'southeast', 'FontSize', 9);
xlim([75, 100]); ylim([0.99, 1.01]);
set(gca, 'LineWidth', 1);

% 子图4: 性能指标对比
subplot(2,3,4);
bar_data = [itae_init, itae_opt; overshoot_init, overshoot_opt; ...
            settling_time_init, settling_time_opt];
b = bar(bar_data, 'grouped');
b(1).FaceColor = color_blue; b(2).FaceColor = color_red;
b(1).EdgeColor = 'none'; b(2).EdgeColor = 'none';
set(gca, 'XTickLabel', {'ITAE', '超调量(%)', '调节时间(s)'});
ylabel('指标值', 'FontSize', 11, 'FontWeight', 'bold');
title('关键性能指标', 'FontSize', 13, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', 'Location', 'northwest', 'FontSize', 9);
grid on; set(gca, 'LineWidth', 1);

% 子图5: PID参数
subplot(2,3,5);
param_data = [init_P, best_params(1); init_I, best_params(2); init_D, best_params(3)];
b = bar(param_data, 'grouped');
b(1).FaceColor = color_blue; b(2).FaceColor = color_red;
b(1).EdgeColor = 'none'; b(2).EdgeColor = 'none';
set(gca, 'XTickLabel', {'K_p', 'K_i', 'K_d'});
ylabel('参数值', 'FontSize', 11, 'FontWeight', 'bold');
title('PID参数对比', 'FontSize', 13, 'FontWeight', 'bold');
legend('优化前', 'GA优化后', 'Location', 'northwest', 'FontSize', 9);
grid on; set(gca, 'LineWidth', 1);
% 添加数值
for i = 1:3
    text(i-0.15, param_data(i,1)*1.1, sprintf('%.2f', param_data(i,1)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9);
    text(i+0.15, param_data(i,2)*1.1, sprintf('%.2f', param_data(i,2)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', color_red);
end

% 子图6: GA收敛曲线
subplot(2,3,6);
if ~isempty(ga_history.generation)
    hold on; box on; grid on;
    plot(ga_history.generation, ga_history.best, '-o', ...
        'Color', color_red, 'LineWidth', 2, 'MarkerSize', 3, 'MarkerFaceColor', color_red);
    xlabel('遗传代数', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('适应度值', 'FontSize', 11, 'FontWeight', 'bold');
    title('GA收敛过程', 'FontSize', 13, 'FontWeight', 'bold');
    xlim([1, max(ga_history.generation)]);
    set(gca, 'LineWidth', 1);
else
    text(0.5, 0.5, '收敛数据不可用', 'HorizontalAlignment', 'center', ...
        'FontSize', 11, 'Units', 'normalized');
end

% 添加底部信息栏
annotation('textbox', [0.05, 0.01, 0.9, 0.04], ...
    'String', sprintf(['双容水箱传递函数: G(s)=1/(25s²+10s+1) | ', ...
                      'GA参数: 种群=%d, 代数=%d, 精英=%d | ', ...
                      '优化耗时: %.1f分钟 | ', ...
                      '最优参数: Kp=%.3f, Ki=%.3f, Kd=%.3f'], ...
        options.PopulationSize, output_ga.generations, options.EliteCount, ...
        elapsed_time/60, best_params), ...
    'FontSize', 9, 'EdgeColor', 'k', 'LineWidth', 1.5, ...
    'BackgroundColor', [0.9, 0.95, 1], 'FontName', 'SimHei');

saveas(fig9, '图9_综合报告图.png');
saveas(fig9, '图9_综合报告图.fig');
fprintf('✓ 已保存: 图9_综合报告图.png\n');

%% 生成摘要结果表
fprintf('\n========================================\n');
fprintf('论文用数据汇总\n');
fprintf('========================================\n\n');

fprintf('【表1：系统性能指标对比】\n');
fprintf('┌─────────────┬──────────┬──────────┬──────────┐\n');
fprintf('│  性能指标   │  优化前  │ GA优化后 │ 改善幅度 │\n');
fprintf('├─────────────┼──────────┼──────────┼──────────┤\n');
fprintf('│ ITAE        │ %8.4f │ %8.4f │ %6.1f%% │\n', itae_init, itae_opt, improvement_data(1));
fprintf('│ 超调量(%%)   │ %8.2f │ %8.2f │ %6.1f%% │\n', overshoot_init, overshoot_opt, improvement_data(2));
fprintf('│ 上升时间(s) │ %8.4f │ %8.4f │ %6.1f%% │\n', rise_time_init, rise_time_opt, improvement_data(3));
fprintf('│ 调节时间(s) │ %8.4f │ %8.4f │ %6.1f%% │\n', settling_time_init, settling_time_opt, improvement_data(4));
fprintf('│ 稳态误差    │ %.2e │ %.2e │ %6.1f%% │\n', steady_error_init, steady_error_opt, improvement_data(5));
fprintf('│ 后期波动    │ %.2e │ %.2e │ %6.1f%% │\n', late_std_init, late_std_opt, improvement_data(6));
fprintf('└─────────────┴──────────┴──────────┴──────────┘\n\n');

fprintf('【表2：PID控制器参数对比】\n');
fprintf('┌────────┬──────────┬──────────┬──────────┐\n');
fprintf('│  参数  │  优化前  │ GA优化后 │ 变化幅度 │\n');
fprintf('├────────┼──────────┼──────────┼──────────┤\n');
fprintf('│  Kp    │ %8.4f │ %8.4f │ %6.1f%% │\n', init_P, best_params(1), (best_params(1)-init_P)/init_P*100);
fprintf('│  Ki    │ %8.4f │ %8.4f │ %6.1f%% │\n', init_I, best_params(2), (best_params(2)-init_I)/init_I*100);
fprintf('│  Kd    │ %8.4f │ %8.4f │ %6.1f%% │\n', init_D, best_params(3), (best_params(3)-init_D)/init_D*100);
fprintf('└────────┴──────────┴──────────┴──────────┘\n\n');

fprintf('【表3：遗传算法优化配置】\n');
fprintf('种群大小: %d\n', options.PopulationSize);
fprintf('最大代数: %d\n', options.MaxGenerations);
fprintf('精英个体数: %d\n', options.EliteCount);
fprintf('交叉概率: %.2f\n', options.CrossoverFraction);
fprintf('选择策略: 锦标赛选择\n');
fprintf('变异策略: 自适应可行域变异\n');
fprintf('实际运行代数: %d\n', output_ga.generations);
fprintf('函数评估次数: %d\n', output_ga.funccount);
fprintf('优化耗时: %.2f分钟\n', elapsed_time/60);
fprintf('最优适应度: %.6f\n\n', best_cost);

%% 清理和保存
save_system(modelName);
fprintf('\n========================================\n');
fprintf('图表生成完成！\n');
fprintf('========================================\n\n');

fprintf('已生成以下文件:\n');
fprintf('1. 图1_系统阶跃响应对比.png - 主要响应曲线\n');
fprintf('2. 图2_跟踪误差对比.png - 误差分析\n');
fprintf('3. 图3_性能指标与参数对比.png - 定量对比\n');
fprintf('4. 图4_GA收敛过程分析.png - 优化过程\n');
fprintf('5. 图5_系统稳定性与频域特性.png - 稳定性分析\n');
fprintf('6. 图6_控制信号对比.png - 控制器输出\n');
fprintf('7. 图7_性能改善百分比.png - 改善效果\n');
fprintf('8. 图8_性能对比表.png - 数据表格\n');
fprintf('9. 图9_综合报告图.png - 综合展示\n');
fprintf('\n所有图片均已保存为.png和.fig格式\n');
fprintf('========================================\n\n');