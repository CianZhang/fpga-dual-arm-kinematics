module top_motion_solver
(
    input wire                  sys_clk,                // 系统时钟输入，50MHz频率    
    input wire                  sys_rst_n,              // 复位信号输入，低电平复位

    output wire                 step_pulse_1,           // 步进电机的脉冲信号输出
    output wire                 direction_1,            // 步进电机的方向信号输出
    output wire                 reached_target_1,       // 目标位置标志位
    
    output wire                 step_pulse_2,           // 步进电机的脉冲信号输出
    output wire                 direction_2,            // 步进电机的方向信号输出
    output wire                 reached_target_2,       // 目标位置标志位
    
    output wire                 step_pulse_3,           // 步进电机的脉冲信号输出
    output wire                 direction_3,            // 步进电机的方向信号输出
    output wire                 reached_target_3,       // 目标位置标志位
    
    output wire                 motion_solver_work_done // 解算运动完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg signed [15:0] x_16;      // x坐标输入
reg signed [15:0] y_16;      // y坐标输入
reg signed [15:0] z_16;      // z坐标输入

reg [30:0] counter;          // 30秒计数器 (需要31位来存储1,500,000,000)
reg [1:0] state;             // 状态计数器，用于切换不同坐标

parameter TIME_30S = 31'd2000_000_00;  // 3秒的周期数 (50MHz * 3s)

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

motion_solver motion_solver_inst(
    .sys_clk(sys_clk),                
    .sys_rst_n(sys_rst_n),             
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
    .motion_solver_work_done(motion_solver_work_done)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        x_16 <= 16'sd0;
        y_16 <= 16'sd0;
        z_16 <= 16'sd0;
        counter <= 31'd0;
        state <= 2'd0;
    end
    else begin
        if (counter < TIME_30S - 1) begin
            counter <= counter + 1;
        end
        else begin
            counter <= 31'd0;  // 重置计数器
            state <= state + 1; // 切换到下一个状态
            
            case (state)
                2'd0: begin  // 第一组坐标
                    x_16 <= 16'sd0;    // 30 * 16
                    y_16 <= 16'sd0;    // 30 * 16
                    z_16 <= -16'sd3200;  // -300 * 16
                end
                2'd1: begin  // 第二组坐标
                    x_16 <= 16'sd1000;    // 30 * 16
                    y_16 <= -16'sd1000;    // 30 * 16
                    z_16 <= -16'sd4800;  // -300 * 16
                end
                2'd2: begin  // 第三组坐标
                    x_16 <= -16'sd1000;   // -30 * 16
                    y_16 <= -16'sd1000;   // -30 * 16
                    z_16 <= -16'sd4800;  // -300 * 16
                end
                2'd3: begin  // 第四组坐标
                    x_16 <= -16'sd1000;   // -30 * 16
                    y_16 <= 16'sd1000;    // 30 * 16
                    z_16 <= -16'sd4800;  // -300 * 16
                end
            endcase
        end
    end
end

endmodule