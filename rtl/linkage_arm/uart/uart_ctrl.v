module uart_ctrl
(
    input  wire              sys_clk,            // 系统时钟50MHz
    input  wire              sys_rst_n,          // 全局复位，低有效
    input  wire              rx,                 // 串口接收数据

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
//****************** Parameter and Internal Signal ********************//
//********************************************************************//

wire signed [7:0]  rx_data1;                      // 接收数据1，8位宽接收电机序号
wire signed [7:0]  rx_data2;                      // 接收数据2，8位宽接收旋转方向
wire signed [31:0] rx_data3;                      // 接收数据3，32位宽接收电机角度
wire               rx_done;                       // 接收完成标志

reg               rotation_direction;
reg signed [31:0] final_angle_1;
reg signed [31:0] final_angle_2;
reg signed [31:0] final_angle_3;
reg signed [31:0] final_angle_4;


//********************************************************************//
//************************* Instantiation ****************************//
//********************************************************************//

uart_rx_top uart_rx_top_inst(  
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),
    .rx(rx),       
    .rx_data1(rx_data1), 
    .rx_data2(rx_data2), 
    .rx_data3(rx_data3), 
    .rx_done(rx_done)   
);

stepper_motor_ctrl_1 stepper_motor_ctrl_1_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_1),
    .step_pulse(step_pulse_1),
    .direction(direction_1),
    .reached_target(reached_target_1)
);

stepper_motor_ctrl_2 stepper_motor_ctrl_2_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_2),
    .turn_direction(rotation_direction),
    .step_pulse(step_pulse_2),
    .direction(direction_2),
    .reached_target(reached_target_2)
);

stepper_motor_ctrl_3 stepper_motor_ctrl_3_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_3),
    .step_pulse(step_pulse_3),
    .direction(direction_3),
    .reached_target(reached_target_3)
);

stepper_motor_ctrl_4 stepper_motor_ctrl_4_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_4),
    .step_pulse(step_pulse_4),
    .direction(direction_4),
    .reached_target(reached_target_4)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        rotation_direction <= 0;
        final_angle_1 <= 0;
        final_angle_2 <= 0;
        final_angle_3 <= 0;
        final_angle_4 <= 0;
    end
else begin
        if (rx_done) begin
            case (rx_data1)
                8'd1: begin
                    final_angle_1 <= rx_data3;
                end
                8'd2: begin
                    rotation_direction <= rx_data2[0]; // 取最低位，8位转1位
                    final_angle_2 <= rx_data3;
                end
                8'd3: begin
                    final_angle_3 <= rx_data3;
                end
                8'd4: begin
                    final_angle_4 <= rx_data3;
                end
                default: begin
                    // 保持当前值
                    rotation_direction <= rotation_direction;
                    final_angle_1 <= final_angle_1;
                    final_angle_2 <= final_angle_2;
                    final_angle_3 <= final_angle_3;
                    final_angle_4 <= final_angle_4;
                end
            endcase
        end
        else begin
            // 当 rx_done 为 0 时，保持当前值
            rotation_direction <= rotation_direction;
            final_angle_1 <= final_angle_1;
            final_angle_2 <= final_angle_2;
            final_angle_3 <= final_angle_3;
            final_angle_4 <= final_angle_4;
        end
    end
end

endmodule
