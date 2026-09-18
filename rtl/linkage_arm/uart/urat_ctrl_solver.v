module uart_ctrl_solver
(
    input  wire              sys_clk,            // 系统时钟50MHz
    input  wire              sys_rst_n,          // 全局复位，低有效
    
    output wire              step_pulse_1,       // 平移步进电机脉冲信号
    output wire              direction_1,        // 平移步进电机方向信号
    output wire              reached_target_1,   // 平移达到目标位置标志位        
    output wire              step_pulse_2,       // 旋转步进电机脉冲信号
    output wire              direction_2,        // 旋转步进电机方向信号
    output wire              reached_target_2,   // 旋转达到目标位置标志位            
    output wire              step_pulse_3,       // 大臂步进电机脉冲信号
    output wire              direction_3,        // 大臂步进电机方向信号
    output wire              reached_target_3,   // 大臂达到目标位置标志位         
    output wire              step_pulse_4,       // 小臂步进电机脉冲信号
    output wire              direction_4,        // 小臂步进电机方向信号
    output wire              reached_target_4    // 小臂达到目标位置标志位
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg  signed [15:0] x;                             // 空间坐标X     
reg  signed [15:0] y;                             // 空间坐标Y
reg  signed [15:0] z;                             // 空间坐标Z

reg  signed [31:0] final_angle_1;                 // 平移角度  
reg                turn_direction;                // 旋转轴方向

wire               work_done;                     // 运动完成信号

//********************************************************************//
//************************* Instantiation ****************************//
//********************************************************************//

stepper_motor_ctrl_1 stepper_motor_ctrl_1_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_1),
    .step_pulse(step_pulse_1),
    .direction(direction_1),
    .reached_target(reached_target_1)
);

motion_solver motion_solver_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .i_x(x), 
    .i_y(y),             
    .i_z(z),             
    .turn_direction(turn_direction),  
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

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        x <= 16'sd0;
        y <= 16'sd0;
        z <= 16'sd0;
        final_angle_1 <= 0;
    end
    else begin
        if(rx_done) begin
            x <= rx_data1;
            y <= rx_data2;
            z <= rx_data3;
        end
    end
end

endmodule
