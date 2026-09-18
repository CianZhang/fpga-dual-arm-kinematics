module motion_ctrl
(
    input wire       sys_clk,             // 系统时钟输入，24MHz频率
    input wire       sys_rst_n,           // 复位信号输入，低电平复位
    input wire [9:0] angle_translation,   // 平移角度输入，0-360度，10位
    input wire [9:0] angle_rotation,      // 旋转角度输入，0-360度，10位
    input wire [9:0] angle_bigarm,        // 大臂角度输入，0-360度，10位
    input wire [9:0] angle_smallarm,      // 小臂角度输入，0-360度，10位
    input wire [8:0] angle_servo,         // 舵机角度输入，0-180度，9位
    input wire       gripper_work_en,     // 抓手工作使能信号

    output wire      step_pulse_trans,    // 平移步进电机脉冲输出
    output wire      direction_trans,     // 平移步进电机方向输出

    output wire      step_pulse_rot,      // 旋转步进电机脉冲输出
    output wire      direction_rot,       // 旋转步进电机方向输出

    output wire      step_pulse_bigarm,   // 大臂步进电机脉冲输出
    output wire      direction_bigarm,    // 大臂步进电机方向输出

    output wire      step_pulse_smallarm, // 小臂步进电机脉冲输出
    output wire      direction_smallarm,  // 小臂步进电机方向输出

    output reg       servo_pwm,           // 舵机PWM实际输出信号
    output reg       all_reached_target   // 输出reg类型的all_reached_target信号
);

wire reached_trans, reached_rot, reached_bigarm, reached_smallarm;

// 实例化平移电机控制模块
stepper_motor_ctrl_1 u_stepper_motor_ctrl_1
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(angle_translation),
    .step_pulse(step_pulse_trans),
    .direction(direction_trans),
    .reached_target(reached_trans)
);

// 实例化旋转电机控制模块
stepper_motor_ctrl_2 u_stepper_motor_ctrl_2
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(angle_rotation),
    .step_pulse(step_pulse_rot),
    .direction(direction_rot),
    .reached_target(reached_rot)
);

// 实例化大臂电机控制模块
stepper_motor_ctrl_34 u_stepper_motor_ctrl_34_bigarm
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(angle_bigarm),
    .step_pulse(step_pulse_bigarm),
    .direction(direction_bigarm),
    .reached_target(reached_bigarm)
);

// 实例化小臂电机控制模块
stepper_motor_ctrl_34 u_stepper_motor_ctrl_34_smallarm
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(angle_smallarm),
    .step_pulse(step_pulse_smallarm),
    .direction(direction_smallarm),
    .reached_target(reached_smallarm)
);

// 时钟边沿触发下更新all_reached_target的值
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        all_reached_target <= 1'b0;
    else
        all_reached_target <= reached_trans && reached_rot && reached_bigarm && reached_smallarm;
end

// 实例化舵机控制模块
wire servo_pwm_internal;

servo_ctrl u_servo_ctrl
(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .servo_angle(angle_servo),
    .servo_pwm(servo_pwm_internal)    // 内部PWM信号
);

// 控制舵机PWM输出信号的时序逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        servo_pwm <= 1'b0;  // 复位时将servo_pwm置为0
    else if (gripper_work_en && all_reached_target)
        servo_pwm <= servo_pwm_internal;  // 当gripper_work_en和all_reached_target为真时输出PWM信号
    else
        servo_pwm <= 1'b0;  // 否则保持servo_pwm为0
end

endmodule
