# 乘法器 IP 配置

原 `ip_multiplier.v` 与其底层实现带有 Pango 专有声明。本仓库不附带这两个厂商源码文件，以下参数来自原工程配置，供本地重新生成。

| 参数 | 原值 |
| --- | --- |
| 模块名 | ip_multiplier |
| ASIZE / BSIZE | 32 / 32 |
| A_SIGNED / B_SIGNED | 1 / 1 |
| 输出宽度 | 64 |
| INREG_EN | 0 |
| PIPEREG_EN_1 / 2 / 3 | 0 / 0 / 0 |
| OUTREG_EN | 0 |
| ASYNC_RST | 1 |
| OPTIMAL_TIMING | 0 |
| GRS_EN | FALSE |
| 端口 | ce, rst, clk, a[31:0], b[31:0], p[63:0] |

当前整理版 PDS 预期文件：

```text
generated/ip_multiplier/ip_multiplier.v
generated/ip_multiplier/rtl/ipml_mult_v1_3_ip_multiplier.v
```

生成目录已被 `.gitignore` 排除。生成器版本可能改变底层文件名或附加依赖，届时应通过 PDS 添加实际生成文件。原配置未启用输入、流水或输出寄存器，不能随意开启流水级而不调整调用状态机。未提供替代行为模型，避免把未经核对的替代实现当作原厂 IP 的验证结果。
