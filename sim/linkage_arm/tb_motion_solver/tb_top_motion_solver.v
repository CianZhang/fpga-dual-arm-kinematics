`timescale 1ns/1ns

module tb_top_motion_solver;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg                 sys_clk;
reg                 sys_rst_n;

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

//glbl Instantiate
glbl glbl();

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //

top_motion_solver uut(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
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
    .reached_target_4(reached_target_4)
);

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

initial begin
    sys_clk = 0;
end

always #2 sys_clk = ~sys_clk;  // 24MHz时钟的近似信号，半周期为21ns

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

initial begin
    // 初始化信号
    sys_clk = 0;
    sys_rst_n = 0;
    
    // 复位
    #20;
    sys_rst_n = 1;
    
    #5000;
    #5000;
    
    #500000000;
    #500000000;
    #500000000;
    
    #500000000;
    #500000000;
    #500000000;
    
    #500000000;
    #500000000;
    #500000000;
    // 停止仿真
    $stop;
end

endmodule
