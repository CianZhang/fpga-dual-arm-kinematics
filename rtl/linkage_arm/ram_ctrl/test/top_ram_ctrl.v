module top_motion_solver
#(
    parameter NUM_TARGETS = 5               // 预设目标位置的数量，可调整
)
(
    input wire         sys_clk,             // 系统时钟
    input wire         sys_rst_n,           // 系统复位，低电平有效
    input wire         start,               // 启动信号

    output wire        step_pulse_1,        // 平移步进电机脉冲信号
    output wire        direction_1,         // 平移步进电机方向信号
    output wire        reached_target_1,    // 平移达到目标位置标志位

    output wire        step_pulse_2,        // 旋转步进电机脉冲信号
    output wire        direction_2,         // 旋转步进电机方向信号
    output wire        reached_target_2,    // 旋转达到目标位置标志位    

    output wire        step_pulse_3,        // 大臂步进电机脉冲信号
    output wire        direction_3,         // 大臂步进电机方向信号
    output wire        reached_target_3,    // 大臂达到目标位置标志位

    output wire        step_pulse_4,        // 小臂步进电机脉冲信号
    output wire        direction_4,         // 小臂步进电机方向信号
    output wire        reached_target_4     // 小臂达到目标位置标志位
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************// 

reg signed [15:0] target_x[15:0];
reg signed [15:0] target_y[15:0];
reg signed [15:0] target_z[15:0];

reg        [15:0] current_target;
wire              motion_done;
reg               motion_done_flag;

initial begin
    target_x[0] = 16'd250;
    target_y[0] = 16'd100;
    target_z[0] = 16'd0;  

    target_x[1] = 16'd250;
    target_y[1] = 16'd300;
    target_z[1] = 16'd0; 

    target_x[2] = 16'd250;  
    target_y[2] = 16'd200;
    target_z[2] = 16'd0;
    
    target_x[3] = 16'd250;  
    target_y[3] = 16'd400;
    target_z[3] = 16'd0;

    target_x[4] = 16'd250;  
    target_y[4] = 16'd0;
    target_z[4] = 16'd0;
end
  
//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

ram_ctrl ram_ctrl_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .start(start),
    .i_x(target_x[current_target]),
    .i_y(target_y[current_target]),
    .i_z(target_z[current_target]),
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
    .motion_done(motion_done)      
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        current_target <= 0;    
        motion_done_flag <= 0;    
    end
    else if (motion_done && !motion_done_flag && start) begin
        current_target <= (current_target < NUM_TARGETS - 1) ? current_target + 1 : 0;
        motion_done_flag <= 1;
    end
    else if (!motion_done) begin
        motion_done_flag <= 0; 
    end
end

endmodule
