`timescale 1ns/1ns

module tb_motion_solver;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg                 sys_clk;
reg                 sys_rst_n;
reg  signed [15:0]  i_x;              // 测试输入的 X 坐标
reg  signed [15:0]  i_y;              // 测试输入的 Y 坐标
reg  signed [15:0]  i_z;              // 测试输入的 Z 坐标

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

wire                work_done;

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //

motion_solver uut(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .i_x(i_x),
    .i_y(i_y),
    .i_z(i_z),
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
    .work_done(work_done)
);

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

GTP_GRS GRS_INST(
    .GRS_N(1'b1)
    );
    
initial begin
    sys_clk = 0;
end

always #10 sys_clk = ~sys_clk;

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

initial begin
    // 初始化信号
    sys_clk = 0;
    sys_rst_n = 0;
    i_x = 0;
    i_y = 0;
    i_z = 0;
    
    // 复位
    #20;
    sys_rst_n = 1;
    
    // 设置第一个测试坐标
    #10;
    i_x = 16'd250;
    i_y = 16'd200;
    i_z = 16'd40;
    
    #500000000;
    #500000000;
    #500000000;
    // 停止仿真
    $stop;
end

endmodule
