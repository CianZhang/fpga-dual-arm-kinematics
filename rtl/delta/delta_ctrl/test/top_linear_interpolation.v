module top_linear_interpolation
(
    input wire                  sys_clk,             // 系统时钟，50MHz
    input wire                  sys_rst_n,           // 系统复位，低电平有效
    input wire                  start,               // 外部按键启动信号
    input wire                  rx,                  // 串口接收坐标
    
    output wire                 step_pulse_1,        // 电机 1 脉冲信号
    output wire                 direction_1,         // 电机 1 方向信号
    output wire                 reached_target_1,    // 电机 1 到达标志
    output wire                 step_pulse_2,        // 电机 2 脉冲信号
    output wire                 direction_2,         // 电机 2 方向信号
    output wire                 reached_target_2,    // 电机 2 到达标志
    output wire                 step_pulse_3,        // 电机 3 脉冲信号
    output wire                 direction_3,         // 电机 3 方向信号
    output wire                 reached_target_3,    // 电机 3 到达标志

    output wire                 step_pulse_tran,     // 传送带电机脉冲信号
    output wire                 direction_tran,      // 传送带电机方向信号
    output wire                 reached_target_tran, // 传送带电机到达标志

    output wire                 move_done            // 插补完成信号    
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg         [31:0]   target_tran;

reg  signed [15:0]   x_16;
reg  signed [15:0]   y_16;
reg  signed [15:0]   z_16;

wire signed [15:0]   x;   
wire signed [15:0]   y;   
wire signed [15:0]   z;   

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

linear_interpolation linear_interp_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .start(start),
    .x_16(x_16),
    .y_16(y_16),
    .z_16(z_16),
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

uart_rx_top uart_rx_top_inst(  
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),
    .rx(rx),       
    .rx_data1(x), 
    .rx_data2(y), 
    .rx_data3(z), 
    .rx_done(rx_done)   
);

tran_tape tran_tape_inst(
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),   
    .target_angles(target_tran),
    .step_pulse(step_pulse_tran),   
    .direction(direction_tran),    
    .reached_target(reached_target_tran)
);


//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        x_16 <= 16'sd0;
        y_16 <= 16'sd0;
        z_16 <= 16'sd0;
        target_tran <= 0;
    end
    else begin
        if(start && rx_done) begin
            x_16 <= x;
            y_16 <= y;
            z_16 <= z;
            target_tran <= 54272;
        end
    end
end

endmodule
