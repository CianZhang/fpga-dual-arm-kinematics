module sqrt (
    input wire          sys_clk,       // 系统时钟输入
    input wire          sys_rst_n,     // 复位信号输入，低电平复位
    input wire [63:0]   data,          // 需要开方的数（改为64位）
    output reg [31:0]   sqrt,          // 开方后的结果（改为32位）
    output reg          valid          // 输出结果有效标志
);

reg [63:0] radicand;                  // 被开方数（改为64位）
reg [31:0] result;                    // 开方结果（改为32位）
reg [31:0] bit;                       // 二进制位，用来进行逐位逼近（改为32位）
reg        busy;                      // 模块是否正在计算
reg        start;                     // 开始信号
reg [63:0] data_reg;                  // 存储上一次输入的 data 用于检测新数据（改为64位）

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        sqrt   <= 32'b0;              // 输出结果宽度改为32位
        valid  <= 1'b0;               // 复位时，valid 置低
        busy   <= 1'b0;
        bit    <= 32'b0;              // bit 宽度改为32位
        result <= 32'b0;              // result 宽度改为32位
        radicand <= 64'b0;            // radicand 宽度改为64位
        start  <= 1'b0;
        data_reg <= 64'b0;            // data_reg 宽度改为64位
    end
    else begin
        // 检测到新数据输入时，重新开始计算
        if (data != data_reg && !busy) begin
            radicand <= data;
            bit <= 32'h8000_0000;     // 从最高位开始逼近（32位最高位）
            result <= 32'b0;
            busy <= 1'b1;
            valid <= 1'b0;            // 在开始计算时将 valid 置低
            start <= 1'b1;
            data_reg <= data;         // 更新上一次的数据记录
        end
        else if (busy) begin
            // 逐位逼近
            if (bit != 0) begin
                if ((result | bit) * (result | bit) <= radicand) begin
                    result <= result | bit;
                end
                bit <= bit >> 1;
            end
            else begin
                // 计算结束，结果已得到
                sqrt <= result;
                valid <= 1'b1;            // 计算完成时将 valid 拉高
                busy <= 1'b0;             // 标记计算结束
                start <= 1'b0;
            end
        end
    end
end

endmodule