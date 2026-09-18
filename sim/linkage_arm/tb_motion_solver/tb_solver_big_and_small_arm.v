`timescale 1ns/1ns

module tb_solver_big_and_small_arm;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg                 sys_clk;
reg                 sys_rst_n;
reg  signed [31:0]  i_x;              // 测试输入的 X 坐标
reg  signed [31:0]  i_y;              // 测试输入的 Y 坐标

wire                step_pulse_3;     // 输出的步进电机脉冲信号
wire                direction_3;      // 输出的步进电机方向信号
wire                reached_target_3; // 步进电机是否达到目标位置

wire                step_pulse_4;     // 输出的步进电机脉冲信号
wire                direction_4;      // 输出的步进电机方向信号
wire                reached_target_4; // 步进电机是否达到目标位置

//glbl Instantiate
glbl glbl();

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //

solver_big_and_small_arm uut (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .i_x(i_x),
    .i_y(i_y),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),
    .reached_target_4(reached_target_4)
);

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

initial begin
    sys_clk = 0;
end

always #21 sys_clk = ~sys_clk;  // 24MHz时钟的近似信号，半周期为21ns

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

initial begin
    // 初始化信号
    sys_clk = 0;
    sys_rst_n = 0;
    i_x = 0;
    i_y = 0;
    
    // 复位
    #20;
    sys_rst_n = 1;
    
    // 设置第一个测试坐标
    #10;
    i_x = 32'd300;
    i_y = 32'd40;
    #5000
//    i_x = 32'd250;
//    i_y = 32'd10;
//    i_x = 32'd250;
//    i_y = 32'd0;

    //#500000000;                    // 等待足够长的时间来模拟电机转动
    //#500000000;

    //i_x = 32'd300;
    //i_y = -32'd150;
    
    //#500000000;                    // 等待足够长的时间来模拟电机转动
    //#500000000;
   
    // 停止仿真
    $stop;
end

endmodule
