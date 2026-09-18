module top_motion_ctrl
(
    input wire        sys_clk,           // 系统时钟输入
    input wire        sys_rst_n,         // 复位信号输入，低电平复位

    output wire       step_pulse_trans,  // 平移步进电机脉冲输出
    output wire       direction_trans,   // 平移步进电机方向输出
    output wire       step_pulse_rot,    // 旋转步进电机脉冲输出
    output wire       direction_rot,     // 旋转步进电机方向输出
    output wire       step_pulse_bigarm, // 大臂步进电机脉冲输出
    output wire       direction_bigarm,  // 大臂步进电机方向输出
    output wire       step_pulse_smallarm, // 小臂步进电机脉冲输出
    output wire       direction_smallarm,  // 小臂步进电机方向输出
    output wire       servo_pwm,         // 舵机PWM信号输出
    output wire       led_all_reached    // LED指示所有位置是否到达目标
);

    // 固定的角度赋值
    wire [9:0] angle_translation = 10'd180;  // 平移角度：90度
    wire [9:0] angle_rotation = 10'd270;    // 旋转角度：180度
    wire [9:0] angle_bigarm = 10'd45;      // 大臂角度：120度
    wire [9:0] angle_smallarm = 10'd30;     // 小臂角度：45度
    wire [8:0] angle_servo = 9'd90;         // 舵机角度：45度

    // 固定的抓手工作使能信号
    wire gripper_work_en = 1'b1;            // 抓手工作始终使能

    // 用于显示所有电机是否到达目标位置的 LED 输出信号
    wire all_reached_target;

    assign led_all_reached = all_reached_target;

    // 实例化 motion_ctrl 模块
    motion_ctrl u_motion_ctrl
    (
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .angle_translation(angle_translation),
        .angle_rotation(angle_rotation),
        .angle_bigarm(angle_bigarm),
        .angle_smallarm(angle_smallarm),
        .angle_servo(angle_servo),
        .gripper_work_en(gripper_work_en),
        .step_pulse_trans(step_pulse_trans),
        .direction_trans(direction_trans),
        .step_pulse_rot(step_pulse_rot),
        .direction_rot(direction_rot),
        .step_pulse_bigarm(step_pulse_bigarm),
        .direction_bigarm(direction_bigarm),
        .step_pulse_smallarm(step_pulse_smallarm),
        .direction_smallarm(direction_smallarm),
        .servo_pwm(servo_pwm),
        .all_reached_target(all_reached_target)
    );

endmodule
