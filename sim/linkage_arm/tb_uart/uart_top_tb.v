`timescale 1ns / 1ps

module uart_top_tb;

// 参数定义
parameter CLK_PERIOD = 20;           // 时钟周期 20ns (50MHz)
parameter UART_BPS   = 9600;         // 波特率
parameter CLK_FREQ   = 50_000_000;   // 时钟频率
parameter BIT_TIME   = CLK_FREQ / UART_BPS; // 每个位的时间（单位：时钟周期）

// 测试信号
reg         sys_clk;
reg         sys_rst_n;
reg         send_trig;
reg         rx;
wire        tx;
wire [3:0] data_out;
wire        data_valid;

// DUT 实例化
uart_top #(
    .UART_BPS(UART_BPS),
    .CLK_FREQ(CLK_FREQ)
) uut (
    .sys_clk    (sys_clk),
    .sys_rst_n  (sys_rst_n),
    .send_trig  (send_trig),
    .rx         (rx),
    .tx         (tx),
    .num_out    (data_out),
    .data_valid (data_valid)
);

// 时钟生成
initial begin
    sys_clk = 0;
    forever #(CLK_PERIOD / 2) sys_clk = ~sys_clk;
end

// 复位和测试序列
initial begin
    // 初始化信号
    sys_rst_n = 0;
    send_trig = 0;
    rx = 1;  // 串口空闲状态为高电平
    #100;    // 等待 100ns
    sys_rst_n = 1;  // 释放复位

    // 测试 1：触发发送
    #1000;          // 等待一段时间
    send_trig = 1;  // 拉高触发信号
    #20;            // 保持一个时钟周期
    send_trig = 0;

    // 等待发送完成，开始模拟接收数据
    #(BIT_TIME * 10 * 8 * CLK_PERIOD);  // 等待 8 字节发送完成（10位/字节）

    // 测试 2：模拟返回数据包 02 0C B0 30 00 04 00 A1 B2 C3 D4 E5
    send_byte(8'h02);  // 起始字节
    send_byte(8'h0C);
    send_byte(8'hB0);
    send_byte(8'h30);
    send_byte(8'h00);
    send_byte(8'h04);
    send_byte(8'h00);
    send_byte(8'hA1);  // 未知数据 1
    send_byte(8'hB2);  // 未知数据 2
    send_byte(8'hC3);  // 未知数据 3
    send_byte(8'hD4);  // 未知数据 4
    send_byte(8'hE5);  // 未知数据 5

    // 等待接收完成并检查输出
    #10000;
    if (data_valid && data_out == 32'hD4C3B2A1) begin
        $display("Test Passed: data_out = %h, data_valid = %b", data_out, data_valid);
    end
    else begin
        $display("Test Failed: data_out = %h, data_valid = %b", data_out, data_valid);
    end

    // 结束仿真
    #10000;
    $stop;
end

// 任务：发送一个字节（含起始位和停止位）
task send_byte;
    input [7:0] data;
    integer i;
    begin
        // 起始位
        rx = 0;
        #(BIT_TIME * CLK_PERIOD);

        // 8 位数据
        for (i = 0; i < 8; i = i + 1) begin
            rx = data[i];
            #(BIT_TIME * CLK_PERIOD);
        end

        // 停止位
        rx = 1;
        #(BIT_TIME * CLK_PERIOD);
    end
endtask

// 监视 tx 输出
initial begin
    $monitor("Time=%0t, tx=%b, data_out=%h, data_valid=%b", $time, tx, data_out, data_valid);
end

endmodule