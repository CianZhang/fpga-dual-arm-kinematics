# Python 参考材料

`delta_inverse.py` 为本次整理新增的标准库命令行入口，沿用原反解的结构尺寸、0.8660 近似与角度分支。默认输入单位为毫米，输出物理电机命令角及 ×256 编码值。

`legacy/delta_inverse_original.py` 是原 `建模/DeltaX/运动学反解.py`，字节内容未改变。

`legacy/delta_forward_original.py` 和 `legacy/delta_workspace_original.py` 仅作为历史参考；其参数不能自动视为最终机械臂尺寸。工作空间脚本存在原有缩进问题，本次没有悄悄修复并把结果当作原比赛证据。若使用这些旧脚本，需另行安装 numpy / matplotlib 并先核对模型。
