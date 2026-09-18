`timescale 1ns / 1ps

module tb_top_linear_interpolation;

// 测试信号定义
reg                  sys_clk;             // 系统时钟，50MHz
reg                  sys_rst_n;           // 系统复位
reg                  start;               // 启动信号
reg                  rx;

wire                 step_pulse_1;        // 电机 1 脉冲信号
wire                 direction_1;         // 电机 1 方向信号
wire                 reached_target_1;    // 电机 1 到达标志
wire                 step_pulse_2;        // 电机 2 脉冲信号
wire                 direction_2;         // 电机 2 方向信号
wire                 reached_target_2;    // 电机 2 到达标志
wire                 step_pulse_3;        // 电机 3 脉冲信号
wire                 direction_3;         // 电机 3 方向信号
wire                 reached_target_3;    // 电机 3 到达标志
wire                 move_done;           // 插补完成信号

GTP_GRS GRS_INST(
    .GRS_N(1'b1)
    );

top_linear_interpolation uut (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .start(start),
    .rx(rx),
    .step_pulse_1(step_pulse_1),
    .direction_1(direction_1),
    .reached_target_1(reached_target_1),
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .move_done(move_done)
);

// 时钟生成：50MHz，周期 20ns
initial begin
    sys_clk = 0;
    forever #10 sys_clk = ~sys_clk;  // 20ns 周期
end

// 测试序列
initial begin
    sys_rst_n = 0;                  // 复位信号初始化为低

    #100;                           // 等待100ns

    sys_rst_n = 1;                  // 释放复位信号

    #200; 
    start = 1;
    rx = 0;
    
    #10000;
    
    #1000;
    $stop;
end

endmodule
