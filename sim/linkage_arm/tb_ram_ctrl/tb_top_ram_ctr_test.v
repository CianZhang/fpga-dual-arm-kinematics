`timescale 1ns/1ns

module tb_top_ram_ctrl_test;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg                 sys_clk;
reg                 sys_rst_n;

reg                 i_stamp;          // 输入时间戳,激光管为低电平表示物体进入
wire        [7:0]   sm_seg;           // 数码管段码输出，低电平有效
wire        [3:0]   sm_bit;           // 数码管位码输出，高电平有效
wire                led;              // 用于指示物体是否到来，当激光对射管无遮挡时，应当亮起

reg                 start;            // 启动信号
wire                step_pulse_1;     // 输出的步进电机脉冲信号
wire                direction_1;      // 输出的步进电机方向信号
wire                reached_target_1; // 步进电机是否达到目标位置
wire                step_pulse_2;     // 输出的步进电机脉冲信号
wire                direction_2;      // 输出的步进电机方向信号
wire                reached_target_2; // 步进电机是否达到目标位置
wire                step_pulse_3;     // 输出的步进电机脉冲信号
wire                direction_3;      // 输出的步进电机方向信号
wire                reached_target_3; // 步进电机是否达到目标位置
wire                step_pulse_4;     // 输出的步进电机脉冲信号
wire                direction_4;      // 输出的步进电机方向信号
wire                reached_target_4; // 步进电机是否达到目标位置

wire                pump;

wire                step_pulse;

//glbl Instantiate
glbl glbl();

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //

top_ram_ctrl_test uut(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),  
    .i_stamp(i_stamp),
    .sm_seg(sm_seg),
    .sm_bit(sm_bit),
    .led(led),
    .start(start),              
    .step_pulse_1(step_pulse_1),
    .direction_1(direction_1),
    .reached_target_1(reached_target_1),
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),
    .reached_target_4(reached_target_4),   
    .pump(pump),               
    .step_pulse(step_pulse)          
);

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

initial begin
    sys_clk = 0;
end

always #2 sys_clk = ~sys_clk;

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

initial begin
    // 初始化信号
    sys_clk = 0;
    sys_rst_n = 0;
    start = 0;
    i_stamp = 1;
    
    // 复位
    #20;
    sys_rst_n = 1;
    
    #50000
    start = 1;
    #5000000;
    i_stamp = 0;
    #5000000;
    i_stamp = 1;
    #5000000;
    i_stamp = 0;
    #5000000;
    i_stamp = 1;
    #5000000;
    i_stamp = 0;
    #5000000;
    i_stamp = 1;
    #5000000;
    i_stamp = 0;  
    #5000000;
    i_stamp = 1; 
    #5000000;
    #5000000;
    #5000000;
    i_stamp = 0;
    #5000000;
    #5000000;
    #5000000;
    #5000000;
    #5000000;
    $stop;
end

endmodule
