//旋转轴步进电机的控制模块，比例为5.5
//加入梯度加减速算法
//turn_direction 0顺 1逆

module stepper_motor_ctrl_2
(
    input wire        sys_clk,         // 系统时钟输入，50MHz频率
    input wire        sys_rst_n,       // 复位信号输入，低电平复位
    input wire [31:0] target_angles,   // 目标角度，32位宽度，范围0到360
    input wire        turn_direction,  // 旋转方向

    output reg        step_pulse,      // 控制步进电机的脉冲信号输出，用于步进控制
    output reg        direction,       // 控制步进电机的方向信号输出（1：正转，0：反转）
    output reg        reached_target   // 目标位置标志位，当达到目标角度时置1
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

parameter MAX_PULSE_PERIOD  = 8000;    // 最大脉冲周期（最低速度）//8000
parameter MIN_PULSE_PERIOD  = 2000;    // 最小脉冲周期（最高速度）//2000
parameter ACC_STEP 			= 1;       // 每步增加或减少的脉冲周期

parameter ANGLE_RATIO2      = 55;      // 定义旋转步进电机的角度比例，用55代表5.5，放大10倍来避免小数点
parameter STEPS_PER_REV 	= 6400;    // 每转一圈需要6400个脉冲（步距角1.8度）

reg               move_flag;           // 标志位，指示电机是否正在运动
reg signed [31:0] current_angles;      // 当前步进电机角度，支持负值
reg        [15:0] pulse_count;         // 脉冲周期计数器，用于产生步进脉冲
reg        [31:0] steps_to_move;       // 实际需要移动的步数
reg        [15:0] steps_accel;         // 加速阶段的步数
reg        [15:0] steps_decel;         // 减速阶段的步数
reg        [15:0] steps_cruise;        // 匀速阶段的步数
reg        [15:0] current_step;        // 当前已经移动的步数
reg        [15:0] pulse_period;        // 当前脉冲周期（动态调整）
reg signed  [63:0] p;                  // 乘法结果

parameter STATE_CAL_ALL = 2'b00;        // 计算总步数
parameter STATE_WAIT    = 2'b01;        // 使用IP核DSP需要等一个时钟
parameter STATE_CAL_ADD = 2'b10;        // 计算加减步数
parameter STATE_MOVE    = 2'b11;        // 电机开始运动

reg        [1:0]  state, next_state;   // 当前状态和下一个状态寄存器

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

// 初始化电机状态
initial begin
    step_pulse = 0;                   // 初始脉冲信号为低电平
    direction = 0;                    // 初始方向为正转
    reached_target = 1;               // 初始时到达目标位置
    current_angles = 0;               // 当前角度初始为0
    pulse_count = 0;                  // 脉冲计数器初始为0
    pulse_period = MAX_PULSE_PERIOD; // 初始为最大脉冲周期（最低速度）
    current_step = 0;                 // 初始步数为0
end

// 状态寄存器逻辑 (同步)
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= STATE_CAL_ALL;           // 复位时进入初始状态
    end 
    else begin
        state <= next_state;           // 每个时钟周期更新状态
    end
end

// 下一个状态逻辑
always @(*) begin
    case (state)
        STATE_CAL_ALL:
            if (move_flag == 0 && target_angles != current_angles)
                next_state <= STATE_WAIT;
            else
                next_state <= STATE_CAL_ALL;
        STATE_WAIT:
            next_state <= STATE_CAL_ADD;
        STATE_CAL_ADD:
            next_state <= STATE_MOVE;
        STATE_MOVE:
            if (move_flag == 0) 
                next_state <= STATE_CAL_ALL;
            else 
                next_state <= STATE_MOVE;
        default: begin
                next_state <= STATE_CAL_ALL;
            end
    endcase
end
    
// 主控制逻辑：每个时钟周期运行
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // 系统复位时，重置所有信号
        step_pulse <= 0;
        direction <= 0;
        reached_target <= 1;
        current_angles <= 0;
        pulse_count <= 0;
        steps_to_move <= 0;
        move_flag <= 0;
        current_step <= 0;
        pulse_period <= MAX_PULSE_PERIOD; 
    end
    else begin
        case(state)
            STATE_CAL_ALL: begin  
                current_step <= 0;           
                if (target_angles > current_angles) begin
                    // 如果目标角度大于当前角度
                    // 25031 = ANGLE_RATIO2 * STEPS_PER_REV * 256 / 3600
                    if(turn_direction == 0) begin
                        p <= 25031 * (current_angles + 92160 - target_angles);
                        direction <= 0;
                    end
                    else begin
                        p <= 25031 * (target_angles - current_angles);
                        direction <= 1;
                    end
                end
                else begin
                    // 如果目标角度小于当前角度
                    //25031 = ANGLE_RATIO2 * STEPS_PER_REV * 256 / 3600
                    if(turn_direction == 1) begin
                        p <= 25031 * (current_angles + 92160 - target_angles);
                        direction <= 1;
                    end
                    else begin
                        p <= 25031 * (current_angles - target_angles);
                        direction <= 0;
                    end
                end
            end
            STATE_WAIT: begin 
                steps_to_move <= (p >> 16);
            end
            STATE_CAL_ADD: begin           
                // 根据总步数决定加速、匀速和减速阶段的步数
                // 2 * (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD) / ACC_STEP
                if (steps_to_move < ((MAX_PULSE_PERIOD - MIN_PULSE_PERIOD) + (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD))) begin
                    // 如果步数太小，只加速到一半总步数再减速
                    steps_accel <= steps_to_move >> 1;
                    steps_decel <= steps_to_move >> 1;
                    steps_cruise <= 0;
                end
                else begin
                    // 正常情况下，加速和减速步数相等，匀速阶段为中间剩余步数
                    // (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD) / ACC_STEP
                    steps_accel <= (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD);
                    steps_decel <= (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD);
                    steps_cruise <= steps_to_move - ((MAX_PULSE_PERIOD - MIN_PULSE_PERIOD) + (MAX_PULSE_PERIOD - MIN_PULSE_PERIOD));
                end
                move_flag <= 1;      // 标志电机开始运动
                reached_target <= 0; // 目标未达到
            end
            STATE_MOVE: begin 
                // 如果电机正在运动
                if (move_flag) begin
                    if (pulse_count < pulse_period) begin
                        // 如果脉冲周期未完成，继续计数
                        pulse_count <= pulse_count + 1;
                    end
                    else begin
                        // 如果脉冲周期结束，产生脉冲并重置计数器
                        pulse_count <= 0;
                        step_pulse <= ~step_pulse;  // 反转脉冲信号，形成脉冲
           
                        if (step_pulse == 1) begin
                            // 每当脉冲上升沿时，移动步数减少
                            if (steps_to_move > 0) begin
                                current_step <= current_step + 1;
                                steps_to_move <= steps_to_move - 1;
           
                                // 加速阶段
                                if (current_step < steps_accel) begin
                                    if (pulse_period > MIN_PULSE_PERIOD) begin
                                        pulse_period <= pulse_period - ACC_STEP;  // 逐步增加速度
                                    end
                                end
                                // 减速阶段
                                else if (steps_to_move <= steps_decel) begin
                                    if (pulse_period < MAX_PULSE_PERIOD) begin
                                        pulse_period <= pulse_period + ACC_STEP;  // 逐步降低速度
                                    end
                                end
                                // 匀速阶段
                                else begin
                                    if(steps_cruise > 0) begin
                                        pulse_period <= MIN_PULSE_PERIOD;  // 保持最高速度
                                    end
                                end
                            end
                            else begin
                                // 如果移动完成，停止运动
                                move_flag <= 0;
                                reached_target <= 1;  // 目标到达标志置1
                                current_angles <= target_angles;  // 更新当前角度
                            end
                        end
                    end
                end
            end
            default: ;
        endcase
    end
end

endmodule
