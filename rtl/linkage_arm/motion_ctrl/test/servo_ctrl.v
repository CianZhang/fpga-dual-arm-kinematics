module servo_ctrl
(
    input sys_clk,            // 24MHz系统时钟输入
    input sys_rst_n,          // 系统复位信号，低电平有效
    input [8:0] servo_angle,  // 舵机旋转角度（0~180度）
    
    output reg servo_pwm      // 舵机的PWM输出
);

// PWM周期为10ms，对应的时钟计数值
localparam PWM_PERIOD = 240_000;  // 10ms对应的时钟周期数
localparam MIN_PULSE = 12_000;    // 最小500us脉宽
localparam MAX_PULSE = 60_000;    // 最大2500us脉宽

reg [19:0] cnt;                   // 计数器，用于生成PWM周期
reg [19:0] pulse_width;           // 脉宽计数器，表示PWM高电平时间

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cnt <= 20'd0;
        servo_pwm <= 1'b0;
    end else begin
        if (cnt < PWM_PERIOD) 
            cnt <= cnt + 1'b1;
        else
            cnt <= 20'd0;
        
        // 根据计数器的值来生成PWM信号
        if (cnt < pulse_width)
            servo_pwm <= 1'b1;
        else
            servo_pwm <= 1'b0;
    end
end

// 将舵机角度转换为对应的脉宽（线性映射）
always @(servo_angle) begin
    pulse_width = MIN_PULSE + ((servo_angle * (MAX_PULSE - MIN_PULSE)) / 180);
end

endmodule
