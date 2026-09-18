# 复现说明

## Python 参考计算

在仓库根目录运行（Python 3，仅需标准库）：

```sh
python3 reference/delta_inverse.py --x 0 --y 0 --z -237.5
python3 reference/legacy/delta_inverse_original.py
python3 tools/check_repository.py
```

第一个入口是整理时新增的命令行工具；第二个是未改动的原始样点脚本。两者仅验证浮点参考计算，不代表 RTL 已通过验证。

## FPGA 集成工程

整理版工程：`fpga/pango/top_delta_ctrl.pds`。

原 PDS 头部记录工具版本 `2022.2-SP4.2`，器件为 Logos / PGL22G / MBG324 / speed grade −6。系统源码注释和原仿真使用 50 MHz 时钟。资料目录中的 2021.1 安装包不作为本工程的实际构建版本依据。

1. 按 `fpga/pango/IP_CONFIGURATION.md` 在本地生成 `ip_multiplier`，放到指定目录。
2. 使用 PDS 打开工程。若新版 IP 生成层级不同，在工程中替换两项 IP 文件引用，并纳入生成工具要求的其余文件。
3. 确认顶层为 `top_delta_ctrl`，输入源码与 `fpga/pango/sources.f` 一致。该清单路径相对于 `fpga/pango/`。
4. 导入原引脚约束 `top_delta_ctrl_IO.fdc`，根据实际板卡、接线和时钟约束核对后再生成下载文件。
5. 如需 ModelSim，设置本机工具路径并编译 / 映射匹配版本的 Pango 仿真库。整理版清除了原机器的工具路径。

厂商 IP 尚未生成时，工程不是开箱即编译的状态。本次只核对了文件路径，不承诺工具版本兼容或时序通过。

## 仿真入口

- 集成系统：`sim/delta/tb_top_delta_ctrl.v`。
- Delta 逆解与电机：`sim/delta/tb_motion_solver.v`。
- 连杆机械臂：`sim/linkage_arm/tb_motion_solver/` 下的原测试文件。

各测试工程应分别选择依赖文件。不要将所有 `test/` 和 `sim/` 文件一次性加入同一个工程，其中存在同名顶层及历史模块版本。测试文件中的 `GTP_GRS` 等原语依赖厂商仿真库。

## CAD

机械文件保持原文件名与相对层级，参见 `mechanical/README.md`。本次未执行 SolidWorks 装配重建或 STEP 导出。不要仅凭仿真坐标范围直接对真实机械臂发送运动命令；零点、限位与接线需和原硬件一致。
