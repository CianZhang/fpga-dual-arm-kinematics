`timescale 1ns/1ns

module tb_motion_solver;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg                 sys_clk;
reg                 sys_rst_n;
reg  signed [15:0]  i_x;             
reg  signed [15:0]  i_y;             
reg  signed [15:0]  i_z;             

wire                step_pulse_1;    
wire                direction_1;     
wire                reached_target_1;

wire                step_pulse_2;    
wire                direction_2;     
wire                reached_target_2;

wire                step_pulse_3;    
wire                direction_3;     
wire                reached_target_3;

wire                work_done;

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //

motion_solver uut(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x_16(i_x),
    .y_16(i_y),
    .z_16(i_z),
    .step_pulse_1(step_pulse_1),
    .direction_1(direction_1),
    .reached_target_1(reached_target_1),
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .motion_solver_work_done(work_done)
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
    i_x = 16'sd0;
    i_y = 16'sd0;
    i_z = -16'sd4000;
    
    #500000000;
    #500000000;

    i_x = 16'sd0;
    i_y = 16'sd0;
    i_z = -16'sd5600;
    
    #500000000;
    #500000000;
    // 停止仿真
    $stop;
end

endmodule
