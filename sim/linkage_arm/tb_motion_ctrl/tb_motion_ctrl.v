`timescale 1ns / 1ns

module tb_motion_ctrl;

// ******************************************************************** //
// ********************* Signal Declarations ************************** //
// ******************************************************************** //

reg sys_clk;
reg sys_rst_n;
reg [9:0] angle_translation;
reg [9:0] angle_rotation;
reg [9:0] angle_bigarm;
reg [9:0] angle_smallarm;
reg [8:0] angle_servo;
reg gripper_work_en;

wire step_pulse_trans;
wire direction_trans;
wire step_pulse_rot;
wire direction_rot;
wire step_pulse_bigarm;
wire direction_bigarm;
wire step_pulse_smallarm;
wire direction_smallarm;
wire servo_pwm;
wire all_reached_target;

// ******************************************************************** //
// ******************** Instantiation of the DUT ********************** //
// ******************************************************************** //
motion_ctrl uut (
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

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

initial begin
    sys_clk = 0;
end

always #21 sys_clk = ~sys_clk;  // 24MHz时钟的近似信号，半周期为21ns

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

initial begin
    // 初始化输入信号
    sys_clk = 0;
    sys_rst_n = 0;
    angle_translation = 10'd0;
    angle_rotation = 10'd0;
    angle_bigarm = 10'd0;
    angle_smallarm = 10'd0;
    angle_servo = 9'd30;
    gripper_work_en = 1'b0;

    // 系统复位信号拉低100ns后解除复位
    #100;
    sys_rst_n = 1;

    // 模拟第一个场景：各个角度设为中间值，抓手不工作
    #200;
    angle_translation = 10'd180;
    angle_rotation = 10'd180;
    angle_bigarm = 10'd90;
    angle_smallarm = 10'd90;
    angle_servo = 9'd90;
    gripper_work_en = 1'b0;  // 抓手未使能，PWM不会输出

    // 模拟第二个场景：各个电机都到达目标位置，抓手工作使能
    #500000000;                                  // 等待足够长的时间来模拟电机转动
    #500000000;
    #500000000;
    #500000000;
    #500000000;
    gripper_work_en = 1'b1;  // 启动抓手
    angle_translation = 10'd30;
    angle_rotation = 10'd40;
    angle_bigarm = 10'd135;
    angle_smallarm = 10'd135;

    // 模拟第三个场景：抓手使能但是电机未到达目标位置
    #500000000;                                  // 等待足够长的时间来模拟电机转动
    #500000000;
    #500000000;
    #500000000;
    #500000000;
    angle_translation = 10'd90;
    angle_rotation = 10'd90;
    angle_bigarm = 10'd90;
    angle_smallarm = 10'd90;

    // 模拟第四个场景：再次到达目标位置，抓手仍在工作
    #500000000;                                  // 等待足够长的时间来模拟电机转动
    #500000000;
    #500000000;
    #500000000;
    #500000000;
    angle_translation = 10'd300;
    angle_rotation = 10'd250;
    angle_bigarm = 10'd120;
    angle_smallarm = 10'd120;

    // 模拟结束
    #500000000;                                  // 等待足够长的时间来模拟电机转动
    #500000000;
    #500000000;
    #500000000;
    #500000000;
    $stop;
end

endmodule
