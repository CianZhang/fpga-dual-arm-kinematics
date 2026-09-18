# 第三方材料与来源说明

## 厂商 FPGA IP

原项目的 `ip_multiplier.v` 和 `ipml_mult_v1_3_ip_multiplier.v` 含 Pango 专有源码声明。本次整理未收录其源代码，替换为本地生成参数说明。FPGA 器件原语与 ModelSim 库由本机 PDS 安装提供。

## 机械参考资料

原 `建模/readme.txt` 留有以下来源线索，本次未在线核验页面及其许可证：

- https://www.thingiverse.com/thing:6422152
- https://www.thingiverse.com/thing:6425752
- https://www.bilibili.com/video/BV18S4y1A76F/

目录还包含 Delta-X 相关模型 / 学习资料。作者确认网络 STL 是参考方案，SolidWorks 源文件为本人建模，个人贡献是补充部分结构。`mechanical/solidworks` 仅收录 SolidWorks 源文件，外部 STL 不随仓库分发。两套机械臂的整体机构方案不声明为个人原创。

## 配套控制例程

原报告 3.7.2 / 3.7.3 附近说明 I²C 实现参考了正点原子例程。仓库保存原文件中的现有注释，收录不等于声明配套控制模块全部独立原创。

## 团队作品与报告

原报告、实物照片和整机证据属于团队项目背景。视觉识别由其他团队成员完成；本仓库个人成果重点为机械结构设计和 FPGA 逆运动学实现。

本次整理没有添加 MIT、Apache 等覆盖全部材料的许可证，也没有移除原文件中的署名。未明确许可的参考模型需确认来源条款后再决定公开分发范围。
