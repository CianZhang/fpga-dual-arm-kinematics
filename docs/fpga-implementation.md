# FPGA 实现

## 阅读顺序

| 功能 | 入口 |
| --- | --- |
| 集成系统顶层 | `rtl/delta/delta_ctrl/top_delta_ctrl.v` |
| Delta 解算调度 | `rtl/delta/motion_solver/motion_solver.v` |
| 坐标预处理 | `rtl/delta/motion_solver/pretreat_data.v` |
| 单关节角度求解 | `rtl/delta/motion_solver/calculate_angle.v` |
| CORDIC 与开方 | 同目录 `cordic_serial.v`、`sqrt.v` |
| 连杆机械臂解算 | `rtl/linkage_arm/motion_solver/arm_motion_solver.v` |
| 大小臂与旋转轴 | 同目录 `solver_big_and_small_arm.v`、`solver_rotation.v` |

## 定点格式

Delta 输入为 16 位有符号坐标编码，物理毫米值乘 16；ABC 系数使用 64 位有符号寄存器，原设计按 ×65536 缩放。CORDIC 角度输出按 ×65536 表示，`calculate_angle` 最终执行 `33280 - (raw_angle >> 8)`，对应 `(130° - θ) ×256`。

这些模块使用混合缩放与截位，不能将所有信号统一描述为某一种 Q 格式。核对误差时需要逐级追踪缩放、符号位、乘法输入截取和输出右移。

## 分时复用与调度

`pretreat_data` 通过状态机将不同操作数送入同一个 32×32 位乘法器实例。`calculate_angle` 使用另一个乘法器实例处理平方和与后续乘积，并调用开方与 CORDIC 模块。Delta 顶层复用单个 `calculate_angle` 实例依次处理三个分支；源码头部“同步解算”不应被解读成三套完全并行的角度计算硬件。

报告 4.1.3 记录预处理 23 周期、单关节计算约 70 周期、完整 Delta 解算 235 周期。报告其他位置也出现“14 周期”等不同描述，因此仓库只把 235 周期作为报告的历史整体数据，不将不同层级的时延混在一起。

## 算法完成与运动完成

当前 `motion_solver` 同时实例化步进电机控制器，外部输出是电机脉冲、方向和完成状态。评估纯算法延迟应观察内部角度结果与计算阶段，不能直接把包含电机运动的完成信号当作逆解延迟。

部分计算单元靠输入变化触发，接口并不是标准 `start/valid/ready` 协议。重复相同输入、复位后零值和连续输入需要专门检查；本次保留原比赛行为，没有借整理机会修改算法接口。

## 工程相关

原集成工程同时引用两类机械臂，但 UART 文件存在多个历史版本。整理版按照集成 PDS 的选择，保留 `fpga/pango/source/uart/` 两个工程局部版本；不要递归导入整个 RTL 目录，否则可能产生同名模块冲突。

乘法器生成参数与文件位置见 [IP 配置说明](../fpga/pango/IP_CONFIGURATION.md)。
