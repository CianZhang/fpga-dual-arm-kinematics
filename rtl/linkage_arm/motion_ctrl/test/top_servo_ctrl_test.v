module top_servo_ctrl_test
(
    input sys_clk,          // 24MHz系统时钟输入
    input reset_button,     // 系统复位按键，低电平有效
    
    output servo_pwm        // 舵机的PWM输出信号
);

// 内部复位信号，按键为低电平有效
wire sys_rst_n;
assign sys_rst_n = reset_button;

// 定义固定的舵机角度值，赋值为90度
wire [8:0] servo_angle;
assign servo_angle = 9'd43; // 90度

// 实例化舵机控制模块
servo_ctrl u_servo_ctrl (
    .sys_clk(sys_clk),            // 系统时钟
    .sys_rst_n(sys_rst_n),        // 系统复位，低电平有效
    .servo_angle(servo_angle),    // 固定的舵机角度信号
    .servo_pwm(servo_pwm)         // 舵机PWM输出信号
);

endmodule
