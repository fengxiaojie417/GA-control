%% create_pid_model.m - 创建PID控制系统模型
% 功能：建立双容水箱的PID控制系统，供GA优化使用

clear; clc; close all;

fprintf('========================================\n');
fprintf('创建PID控制系统模型\n');
fprintf('========================================\n\n');

%% 1. 创建新模型
modelName = 'pid_tank_model';

% 清理旧模型
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

% 创建新模型
new_system(modelName);
open_system(modelName);

% 设置仿真参数
set_param(modelName, 'StopTime', '100');
set_param(modelName, 'SolverType', 'Variable-step');
set_param(modelName, 'Solver', 'ode45');

fprintf('✓ 模型创建成功: %s\n', modelName);

%% 2. 添加设定值（阶跃信号）
add_block('simulink/Sources/Step', [modelName '/Setpoint']);
set_param([modelName '/Setpoint'], ...
    'Time', '0', ...
    'Before', '0', ...
    'After', '1', ...
    'Position', [50 195 80 225]);

fprintf('✓ 设定值模块添加完成\n');

%% 3. 添加PID控制器
add_block('simulink/Continuous/PID Controller', [modelName '/PID_Controller']);
set_param([modelName '/PID_Controller'], ...
    'Controller', 'PID', ...
    'P', '5', ...
    'I', '0.5', ...
    'D', '1', ...
    'InitialConditionForIntegrator', '0', ...
    'Position', [300 185 400 235]);

fprintf('✓ PID控制器添加完成\n');

%% 4. 添加被控对象（双容水箱传递函数）
% 传递函数: G(s) = 1 / (25s^2 + 10s + 1)
% 这是一个典型的二阶系统，模拟双容水箱的动态特性
add_block('simulink/Continuous/Transfer Fcn', [modelName '/Plant']);
set_param([modelName '/Plant'], ...
    'Numerator', '[1]', ...
    'Denominator', '[25 10 1]', ...
    'Position', [500 190 600 230]);

fprintf('✓ 被控对象添加完成\n');

%% 5. 添加误差求和节点
add_block('simulink/Math Operations/Sum', [modelName '/Error_Sum']);
set_param([modelName '/Error_Sum'], ...
    'Inputs', '+-', ...
    'Position', [200 195 220 215]);

%% 6. 添加示波器
add_block('simulink/Sinks/Scope', [modelName '/Scope']);
set_param([modelName '/Scope'], ...
    'Position', [700 185 730 235]);

% 配置示波器显示两条曲线
scopeConfig = get_param([modelName '/Scope'], 'ScopeConfiguration');
scopeConfig.NumInputPorts = '2';
scopeConfig.LayoutDimensions = [2 1];

fprintf('✓ 示波器添加完成\n');

%% 7. 添加数据记录模块
% 记录误差信号
add_block('simulink/Sinks/To Workspace', [modelName '/Save_Error']);
set_param([modelName '/Save_Error'], ...
    'VariableName', 'error_data', ...
    'SaveFormat', 'Array', ...
    'Position', [700 80 750 110]);

% 记录输出信号
add_block('simulink/Sinks/To Workspace', [modelName '/Save_Output']);
set_param([modelName '/Save_Output'], ...
    'VariableName', 'output_data', ...
    'SaveFormat', 'Array', ...
    'Position', [700 120 750 150]);

% 记录时间
add_block('simulink/Sinks/To Workspace', [modelName '/Save_Time']);
set_param([modelName '/Save_Time'], ...
    'VariableName', 'time_data', ...
    'SaveFormat', 'Array', ...
    'Position', [700 280 750 310]);

add_block('simulink/Sources/Clock', [modelName '/Clock']);
set_param([modelName '/Clock'], ...
    'Position', [630 285 660 305]);

fprintf('✓ 数据记录模块添加完成\n');

%% 8. 添加信号分支点
% 用于将输出信号分配到多个目的地
add_block('simulink/Signal Routing/Goto', [modelName '/Output_Goto']);
set_param([modelName '/Output_Goto'], ...
    'GotoTag', 'system_output', ...
    'Position', [650 195 680 215]);

add_block('simulink/Signal Routing/From', [modelName '/Output_From']);
set_param([modelName '/Output_From'], ...
    'GotoTag', 'system_output', ...
    'Position', [150 215 180 235]);

%% 9. 连接所有信号线
% 主控制回路
add_line(modelName, 'Setpoint/1', 'Error_Sum/1', 'autorouting', 'on');
add_line(modelName, 'Error_Sum/1', 'PID_Controller/1', 'autorouting', 'on');
add_line(modelName, 'PID_Controller/1', 'Plant/1', 'autorouting', 'on');
add_line(modelName, 'Plant/1', 'Output_Goto/1', 'autorouting', 'on');

% 反馈连接
add_line(modelName, 'Output_From/1', 'Error_Sum/2', 'autorouting', 'on');

% 示波器连接
add_line(modelName, 'Setpoint/1', 'Scope/1', 'autorouting', 'on');
add_line(modelName, 'Plant/1', 'Scope/2', 'autorouting', 'on');

% 数据记录连接
add_line(modelName, 'Error_Sum/1', 'Save_Error/1', 'autorouting', 'on');
add_line(modelName, 'Plant/1', 'Save_Output/1', 'autorouting', 'on');
add_line(modelName, 'Clock/1', 'Save_Time/1', 'autorouting', 'on');

fprintf('✓ 信号连接完成\n');

%% 10. 美化布局
% 自动排列模块位置
Simulink.BlockDiagram.arrangeSystem(modelName);

%% 11. 保存模型
save_system(modelName);
fprintf('\n✓ 模型已保存: %s.slx\n', modelName);

% 关闭模型
close_system(modelName, 0);

fprintf('\n========================================\n');
fprintf('✓ 模型创建完成！\n');
fprintf('========================================\n');
fprintf('\n模型特点:\n');
fprintf('1. 被控对象: 双容水箱 G(s)=1/(25s²+10s+1)\n');
fprintf('2. 控制器: PID控制器（Kp, Ki, Kd待优化）\n');
fprintf('3. 设定值: 单位阶跃信号\n');
fprintf('4. 仿真时间: 100秒\n');
fprintf('\n下一步:\n');
fprintf('运行: run_ga_optimize\n');
fprintf('========================================\n\n');
