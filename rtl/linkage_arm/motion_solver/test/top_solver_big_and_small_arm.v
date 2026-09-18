module top_solver_big_and_small_arm
(
    input wire         sys_clk,       // 系统时钟
    input wire         sys_rst_n,     // 系统复位，低电平有效

    // 步进电机控制输出
    output wire        step_pulse_3,   // 大臂步进电机脉冲信号
    output wire        direction_3,    // 大臂步进电机方向信号
    output wire        reached_target_3, // 大臂达到目标位置标志位

    output wire        step_pulse_4,   // 小臂步进电机脉冲信号
    output wire        direction_4,    // 小臂步进电机方向信号
    output wire        reached_target_4  // 小臂达到目标位置标志位
);

// 定义多个X和Y坐标值，用于切换
reg signed [31:0] i_x;    // 当前输入X坐标
reg signed [31:0] i_y;    // 当前输入Y坐标

// 延时计数器
reg [31:0] delay_counter;
reg delay_active;

// 状态机
reg        [2:0]  state;  // 0: 正常状态, 1: 等待延时完成状态

// 定义一些预定的X和Y值
reg signed [31:0] preset_x [1:0];  // 预定的X坐标值
reg signed [31:0] preset_y [1:0];  // 预定的Y坐标值

// 实例化臂角度计算模块
solver_big_and_small_arm solver_inst
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),

    .i_x(i_x),    // 输入的X坐标
    .i_y(i_y),    // 输入的Y坐标

    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),

    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),
    .reached_target_4(reached_target_4)
);

// 初始化固定的X和Y值
initial begin
    i_x = 0;
    i_y = 0;
    state = 0;
    delay_counter = 0;
end

// 时钟驱动逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        // 复位
        i_x <= 0;
        i_y <= 0;
        state <= 0;
        delay_counter <= 0;
        delay_active <= 0;
    end else begin
        case (state)
            0: begin  // 正常状态
                i_x <= 32'd200;
        		i_y <= 32'd45;
              	state <= 1;
                delay_counter <= 0;
            end
            1: begin  // 延时状态
       			if (reached_target_3 && reached_target_4) begin
                    // 当两个reached_target信号都拉高时，启动延时计数
                    if (delay_counter < 72_000_000) begin
                        delay_counter <= delay_counter + 1;
                    end
                    else begin
                    state <= 2;
                    end
                end
            end
            2: begin  // 第二个坐标
            	i_x <= 32'd200;
                i_y <= -32'd30;
                delay_counter <= 0;
                state <= 3;  // 计时
            end
            3: begin  // 延时状态
       			if (reached_target_3 && reached_target_4) begin
                    // 当两个reached_target信号都拉高时，启动延时计数
                    if (delay_counter < 72_000_000) begin
                        delay_counter <= delay_counter + 1;
                    end
                    else begin
                    state <= 0;
                    end
                end
            end
        endcase
    end
end

endmodule
