// 步进电机复位

module stepper_motor_reset
(
    input wire        sys_clk,          // 系统时钟输入，50MHz频率
    input wire        sys_rst_n,        // 复位信号输入，低电平复位
    
    input wire        start_reset,      // 电机开始复位信号
    
    input wire        limit_switch_1,   // 电机1限位开关信号
    input wire        limit_switch_2,   // 电机2限位开关信号
    input wire        limit_switch_3,   // 电机3限位开关信号
    
    output reg        step_pulse_1,     // 电机 1 脉冲信号
    output reg        direction_1,      // 电机 1 方向信号
    output reg        step_pulse_2,     // 电机 2 脉冲信号
    output reg        direction_2,      // 电机 2 方向信号
    output reg        step_pulse_3,     // 电机 3 脉冲信号
    output reg        direction_3,      // 电机 3 方向信号
    
    output reg        finish_reset      // 复位完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

parameter CLK_FREQ = 50_000_000;       // 系统时钟频率 50MHz
parameter STEP_FREQ = 2000;            // 步进电机脉冲频率 1kHz
parameter CNT_MAX = CLK_FREQ / (2 * STEP_FREQ); // 脉冲计数器最大值

parameter IDLE      = 2'b00;           // 空闲状态
parameter RESETTING = 2'b01;           // 复位进行状态
parameter FINISH    = 2'b10;           // 复位完成状态

reg [1:0]   state;                     // 状态机当前状态
reg [31:0]  pulse_cnt;                 // 脉冲计数器
reg         pulse_toggle;              // 脉冲翻转信号
reg         motor1_active;             // 电机1运行标志
reg         motor2_active;             // 电机2运行标志
reg         motor3_active;             // 电机3运行标志

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= IDLE;
        finish_reset <= 1'b0;
        motor1_active <= 1'b0;
        motor2_active <= 1'b0;
        motor3_active <= 1'b0;
        direction_1 <= 1'b0;
        direction_2 <= 1'b0;
        direction_3 <= 1'b0;
    end
    else begin
        case (state)
            IDLE: begin
                finish_reset <= 1'b0;
                if (start_reset) begin
                    state <= RESETTING;
                    motor1_active <= 1'b1;
                    motor2_active <= 1'b1;
                    motor3_active <= 1'b1;
                    direction_1 <= 1'b0;
                    direction_2 <= 1'b0;
                    direction_3 <= 1'b0;
                end
            end
            RESETTING: begin
                // 检查限位开关，停止对应电机
                if (!limit_switch_1)
                    motor1_active <= 1'b0;
                if (!limit_switch_2)
                    motor2_active <= 1'b0;
                if (!limit_switch_3)
                    motor3_active <= 1'b0;
                // 所有电机停止时进入完成状态
                if (!motor1_active && !motor2_active && !motor3_active)
                    state <= FINISH;
            end
            FINISH: begin
                finish_reset <= 1'b1;
                if (!start_reset)
                    state <= IDLE; // 等待start_reset拉低以复位状态机
            end
            default: state <= IDLE;
        endcase
    end
end

// 脉冲生成计数器
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pulse_cnt <= 32'd0;
        pulse_toggle <= 1'b0;
    end
    else begin
        if (state == RESETTING) begin
            if (pulse_cnt < CNT_MAX - 1)
                pulse_cnt <= pulse_cnt + 1;
            else begin
                pulse_cnt <= 32'd0;
                pulse_toggle <= ~pulse_toggle;
            end
        end
        else begin
            pulse_cnt <= 32'd0;
            pulse_toggle <= 1'b0;
        end
    end
end

// 脉冲信号输出
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        step_pulse_1 <= 1'b0;
        step_pulse_2 <= 1'b0;
        step_pulse_3 <= 1'b0;
    end
    else begin
        step_pulse_1 <= motor1_active ? pulse_toggle : 1'b0;
        step_pulse_2 <= motor2_active ? pulse_toggle : 1'b0;
        step_pulse_3 <= motor3_active ? pulse_toggle : 1'b0;
    end
end

endmodule
