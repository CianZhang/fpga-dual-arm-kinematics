module top_stepper_motor_ctrl_test
(
    input wire sys_clk,            // 系统时钟输入
    input wire sys_rst_btn,        // 板上的复位按键
    
    output wire step_pulse,        // 步进脉冲输出
    output wire direction,         // 方向
    output wire reached_target     // 是否达到目标位置
);

//********************************************************************//
//****************** Internal Signal Declaration *********************//
//********************************************************************//

wire sys_rst_n;                    // 系统复位信号，低电平复位
reg [31:0] target_angles;           // 目标角度

// 将复位按钮信号取反，按钮按下时为低电平复位
assign sys_rst_n = ~sys_rst_btn;

// 在复位释放后的某个时刻设置目标角度
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // 如果复位有效，重置目标角度
        target_angles <= 32'd0;
    end
    else begin
        // 在系统运行时设定目标角度
        target_angles <= 32'd23040; // 设定目标角度为90度，值可在此处调整
    end
end

// 实例化步进电机控制模块
stepper_motor_ctrl_3 stepper_motor_ctrl_3_inst 
(
    .sys_clk(sys_clk),            // 连接系统时钟
    .sys_rst_n(sys_rst_n),        // 连接复位信号
    .target_angles(target_angles),// 连接目标角度

    .step_pulse(step_pulse),      // 连接步进脉冲信号
    .direction(direction),        // 连接方向信号
    .reached_target(reached_target)  // 连接目标达成信号
);

endmodule
