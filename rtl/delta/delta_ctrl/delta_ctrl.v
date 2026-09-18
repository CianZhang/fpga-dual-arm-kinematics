// 放置1到3号坐标顺序：1六边形、2圆形、3正方形
// 颜色顺序：1红、2绿、3蓝、4黄
// 面朝传送带传输方向，格子代表数字如下
// 1   2   3   4
// 5   6   7   8
// 9   10  11  12

module delta_ctrl
(
    input  wire                 sys_clk,                // 系统时钟，50MHz
    input  wire                 sys_rst_n,              // 系统复位，低电平有效
    
    input  wire                 start_inst,             // 启动初始化机械臂
    input  wire                 start_move,             // 启动运动信号

    input  wire                 data_valid,             // 输入的坐标信号有效
    
    input  wire                 limit_switch_1,         // 电机1限位开关信号
    input  wire                 limit_switch_2,         // 电机2限位开关信号
    input  wire                 limit_switch_3,         // 电机3限位开关信号
    
    input  wire        [3:0]    ltem_red_1,             // 红色六边形
    input  wire        [3:0]    ltem_red_2,             // 红色圆形
    input  wire        [3:0]    ltem_red_3,             // 红色正方形
    input  wire        [3:0]    ltem_green_1,           // 绿色六边形
    input  wire        [3:0]    ltem_green_2,           // 绿色圆形
    input  wire        [3:0]    ltem_green_3,           // 绿色正方形
    input  wire        [3:0]    ltem_blue_1,            // 蓝色六边形
    input  wire        [3:0]    ltem_blue_2,            // 蓝色圆形
    input  wire        [3:0]    ltem_blue_3,            // 蓝色正方形
    input  wire        [3:0]    ltem_yellow_1,          // 黄色六边形
    input  wire        [3:0]    ltem_yellow_2,          // 黄色圆形
    input  wire        [3:0]    ltem_yellow_3,          // 黄色正方形   

    output wire                 Delta_step_pulse_1,     // Delta机械臂电机 1 脉冲信号
    output wire                 Delta_direction_1,      // Delta机械臂电机 1 方向信号
    output wire                 Delta_step_pulse_2,     // Delta机械臂电机 2 脉冲信号
    output wire                 Delta_direction_2,      // Delta机械臂电机 2 方向信号
    output wire                 Delta_step_pulse_3,     // Delta机械臂电机 3 脉冲信号
    output wire                 Delta_direction_3,      // Delta机械臂电机 3 方向信号

    output wire                 step_pulse_1,           // 平移步进电机脉冲信号
    output wire                 direction_1,            // 平移步进电机方向信号
    output wire                 step_pulse_2,           // 旋转步进电机脉冲信号
    output wire                 direction_2,            // 旋转步进电机方向信号
    output wire                 step_pulse_3,           // 大臂步进电机脉冲信号
    output wire                 direction_3,            // 大臂步进电机方向信号
    output wire                 step_pulse_4,           // 小臂步进电机脉冲信号
    output wire                 direction_4,            // 小臂步进电机方向信号

    output reg                  electromagnet,          // 电磁铁打开信号，高电平有效    
    output reg                  sucker_pump,            // 吸盘的泵控制信号
    output reg                  sucker_valve,           // 吸盘的电磁阀控制信号

    output wire                 tx,                     // UART发送输出
    input  wire                 rx                      // 串口接收数据
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

parameter item_wait_z    = -16'sd4900;         // 物体上方Z坐标
parameter item_take_z    = -16'sd5028;         // 物体吸取Z坐标
parameter item_up_z      = -16'sd4000;         // 物体吸取后Z坐标
   
parameter pump_stay_time = 30000000;           // 吸盘停留时间10000000
parameter pump_put_time  = 80000000;           // 吸盘停留时间40000000               

parameter signed [15:0] target_item_put_x [0:2] = {
    -16'sd2300,                                // 物体1 x
    -16'sd2300,                                // 物体2 x
    -16'sd2300                                 // 物体3 x
};
parameter signed [15:0] target_item_put_y [0:2] = {
    -16'sd650,                                 // 物体1 y
     16'sd50,                                  // 物体2 y
     16'sd750                                  // 物体3 y
};
parameter signed [15:0] target_item_put_z [0:2] = {
    -16'sd4800,                                // 物体1 z
    -16'sd4800,                                // 物体2 z
    -16'sd4800                                 // 物体3 z
};

parameter signed [15:0] target_item_take_x [0:12] = {
     16'sd0,                                   // 空 x
    -16'sd666,                                 // 格子1 x
    -16'sd666,                                 // 格子2 x 
    -16'sd666,                                 // 格子3 x
    -16'sd666,                                 // 格子4 x
    
     16'sd0,                                   // 格子5 x
     16'sd0,                                   // 格子6 x
     16'sd0,                                   // 格子7 x
     16'sd0,                                   // 格子8 x
    
     16'sd666,                                 // 格子9 x
     16'sd666,                                 // 格子10 x
     16'sd666,                                 // 格子11 x
     16'sd666                                  // 格子12 x
};
parameter signed [15:0] target_item_take_y [0:12] = {
     16'sd0,                                   // 空 y
    -16'sd1000,                                // 格子1 y
    -16'sd330,                                 // 格子2 y
     16'sd330,                                 // 格子3 y
     16'sd1000,                                // 格子4 y
     
    -16'sd1000,                                // 格子5 y
    -16'sd330,                                 // 格子6 y
     16'sd330,                                 // 格子7 y
     16'sd1000,                                // 格子8 y
     
    -16'sd1000,                                // 格子9 y
    -16'sd330,                                 // 格子10 y
     16'sd330,                                 // 格子11 y
     16'sd1000                                 // 格子12 y
};
parameter signed [15:0] target_item_take_z [0:12] = {
     16'sd0,                                   // 空 z
    -16'sd4580,                                // 格子1 z
    -16'sd4580,                                // 格子2 z
    -16'sd4580,                                // 格子3 z
    -16'sd4580,                                // 格子4 z
    
    -16'sd4580,                                // 格子5 z
    -16'sd4580,                                // 格子6 z
    -16'sd4580,                                // 格子7 z
    -16'sd4580,                                // 格子8 z
    
    -16'sd4580,                                // 格子9 z
    -16'sd4580,                                // 格子10 z
    -16'sd4580,                                // 格子11 z
    -16'sd4580                                 // 格子12 z
};

reg signed [15:0] target_x;
reg signed [15:0] target_y;
reg signed [15:0] target_z;
reg        [31:0] delay_cnt;

reg               catch_finish;

reg               start_reset;
wire              finish_reset;

reg        [3:0]  ltem_cnt;
reg               ltem_cnt_flag;
wire              motion_done;

reg        [7:0]  pi_data;
reg               pi_flag;
wire              tx_done;

parameter STATE_0  = 5'd0;                           
parameter STATE_1  = 5'd1;                
parameter STATE_2  = 5'd2;                      
parameter STATE_3  = 5'd3;            
parameter STATE_4  = 5'd4;                           
parameter STATE_5  = 5'd5;
parameter STATE_6  = 5'd6;
parameter STATE_7  = 5'd7;
parameter STATE_8  = 5'd8;
parameter STATE_9  = 5'd9;
parameter STATE_10 = 5'd10;
parameter STATE_11 = 5'd11;
parameter STATE_12 = 5'd12;
parameter STATE_13 = 5'd13;

reg  [4:0]  state, next_state; 
reg         flag_0_to_1;
reg         flag_1_to_2; 
reg         flag_2_to_3; 
reg         flag_3_to_4;
reg         flag_4_to_5;
reg         flag_5_to_6;
reg         flag_6_to_7;
reg         flag_7_to_8;
reg         flag_8_to_9;
reg         flag_9_to_10;
reg         flag_10_to_11;
reg         flag_11_to_12;
reg         flag_12_to_13;
reg         flag_13_to_0; 

//********************************************************************//
//****************** 平行四边形连杆机械臂的控制逻辑 ******************//
//********************************************************************//

reg  signed [15:0] arm_target_x;
reg  signed [15:0] arm_target_y;
reg  signed [15:0] arm_target_z;
reg         [31:0] arm_delay_cnt;

reg                Delta_start;

reg         [3:0]  plate_cnt;
reg                plate_cnt_flag;

reg                turn_direction; 
reg         [31:0] stepper_motor_1_angles;
wire               arm_motion_done;

wire        [2:0]  match_id;
reg         [2:0]  match_id_pre;

parameter S_0  = 5'd0;
parameter S_1  = 5'd1;
parameter S_2  = 5'd2;
parameter S_3  = 5'd3;
parameter S_4  = 5'd4;
parameter S_5  = 5'd5;
parameter S_6  = 5'd6;
parameter S_7  = 5'd7;
parameter S_8  = 5'd8;
parameter S_9  = 5'd9;
parameter S_10 = 5'd10;
parameter S_11 = 5'd11;
parameter S_12 = 5'd12;
parameter S_13 = 5'd13;
parameter S_14 = 5'd14;
parameter S_15 = 5'd15;
parameter S_16 = 5'd16;
parameter S_17 = 5'd17;
parameter S_18 = 5'd18;
parameter S_19 = 5'd19;
parameter S_20 = 5'd20;
parameter S_21 = 5'd21;
parameter S_22 = 5'd22;
parameter S_23 = 5'd23;
parameter S_24 = 5'd24;
parameter S_25 = 5'd25;
parameter S_26 = 5'd26;

reg  [4:0]  s, next_s; 
reg         f_0_to_1;
reg         f_1_to_2; 
reg         f_2_to_3; 
reg         f_3_to_4;
reg         f_4_to_5;
reg         f_5_to_6;
reg         f_6_to_7;
reg         f_7_to_8;
reg         f_8_to_9;
reg         f_9_to_10;
reg         f_10_to_11;
reg         f_11_to_12;
reg         f_12_to_13;
reg         f_13_to_14;
reg         f_14_to_15;
reg         f_15_to_16;
reg         f_16_to_17;
reg         f_17_to_18;
reg         f_18_to_19;
reg         f_19_to_20;
reg         f_20_to_21;
reg         f_21_to_22;
reg         f_22_to_23;
reg         f_23_to_24;
reg         f_24_to_25;
reg         f_25_to_26;
reg         f_26_to_0;

wire                 Delta_reached_target_1;
wire                 Delta_reached_target_2;
wire                 Delta_reached_target_3;
wire                 reached_target_1;
wire                 reached_target_2;      
wire                 reached_target_3;      
wire                 reached_target_4;      

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

// 上升沿检测和模块复位信号
reg  finish_reset_dly;  // 存储上一周期的 finish_reset
wire finish_reset_pos;  // finish_reset 上升沿
reg  internal_rst_n;    // 内部生成的复位信号，低电平有效
// 上升沿检测
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        finish_reset_dly <= 1'b0;
    end else begin
        finish_reset_dly <= finish_reset;
    end
end
assign finish_reset_pos = finish_reset & ~finish_reset_dly;
// 内部复位信号生成
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        internal_rst_n <= 1'b0;
    end else if (finish_reset_pos) begin
        internal_rst_n <= 1'b0; // 检测到上升沿，触发复位
    end else begin
        internal_rst_n <= 1'b1; // 正常状态保持高电平
    end
end

wire Delta_step_pulse_1_ctrl;
wire Delta_step_pulse_2_ctrl;
wire Delta_step_pulse_3_ctrl;
wire Delta_direction_1_ctrl;
wire Delta_direction_2_ctrl;
wire Delta_direction_3_ctrl;

wire step_pulse_1_reset;
wire step_pulse_2_reset;
wire step_pulse_3_reset;
wire direction_1_reset;
wire direction_2_reset;
wire direction_3_reset;

assign Delta_step_pulse_1 = Delta_step_pulse_1_ctrl | step_pulse_1_reset;
assign Delta_step_pulse_2 = Delta_step_pulse_2_ctrl | step_pulse_2_reset;
assign Delta_step_pulse_3 = Delta_step_pulse_3_ctrl | step_pulse_3_reset;
assign Delta_direction_1  = Delta_direction_1_ctrl  | direction_1_reset ;
assign Delta_direction_2  = Delta_direction_2_ctrl  | direction_2_reset ;
assign Delta_direction_3  = Delta_direction_3_ctrl  | direction_3_reset ;

linear_interpolation linear_interp_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n & internal_rst_n), // 使用组合复位信号
    .start(Delta_start),
    .x_16(target_x),
    .y_16(target_y),
    .z_16(target_z),
    .step_pulse_1(Delta_step_pulse_1_ctrl),
    .direction_1(Delta_direction_1_ctrl),
    .reached_target_1(Delta_reached_target_1),
    .step_pulse_2(Delta_step_pulse_2_ctrl),
    .direction_2(Delta_direction_2_ctrl),
    .reached_target_2(Delta_reached_target_2),
    .step_pulse_3(Delta_step_pulse_3_ctrl),
    .direction_3(Delta_direction_3_ctrl),
    .reached_target_3(Delta_reached_target_3),
    .move_done(motion_done)
);

stepper_motor_reset stepper_motor_reset_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n & internal_rst_n), // 使用组合复位信号
    .start_reset(start_reset),
    .limit_switch_1(limit_switch_1),
    .limit_switch_2(limit_switch_2),
    .limit_switch_3(limit_switch_3),
    .step_pulse_1(step_pulse_1_reset),
    .direction_1(direction_1_reset),
    .step_pulse_2(step_pulse_2_reset),
    .direction_2(direction_2_reset),
    .step_pulse_3(step_pulse_3_reset),
    .direction_3(direction_3_reset),  
    .finish_reset(finish_reset)
);

ram_ctrl_inst ram_ctrl_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .start_inst(start_inst),
    .start_move(start_move),
    .i_x(arm_target_x),
    .i_y(arm_target_y),
    .i_z(arm_target_z),
    .turn_direction(turn_direction),
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),
    .reached_target_4(reached_target_4),
    .motion_done(arm_motion_done)   
);

stepper_motor_ctrl_1 stepper_motor_ctrl_1_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),    
    .target_angles(stepper_motor_1_angles), 
    .step_pulse(step_pulse_1),    
    .direction(direction_1),     
    .reached_target(reached_target_1) 
);       

uart_tx #(
	.CLK_FRE(50),
	.BAUD_RATE(9600)
) u_uart_tx (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .tx_data(pi_data),
    .tx_data_valid(pi_flag),
    .tx_data_ready(tx_done),
    .tx_pin(tx)
);

RFID_uart RFID_uart_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n), 
    .rx(rx),       
    .match_id(match_id)  
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
    
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= STATE_0;             // 复位时，进入初始状态
    end else begin
        state <= next_state;          // 正常情况下，状态按时钟更新
    end
end

always @(*) begin
    case (state)
        STATE_0: begin
            if(flag_0_to_1 == 1)
                next_state = STATE_1; // 如果flag_0_to_1 == 1，转到STATE_1
            else 
                next_state = STATE_0; // 否则保持在STATE_0
        end
        STATE_1: begin
            if(flag_1_to_2 == 1) 
                next_state = STATE_2; // 如果flag_1_to_2 == 1，转到STATE_2
            else 
                next_state = STATE_1; // 否则保持在STATE_1
        end
        STATE_2: begin
            if(flag_2_to_3 == 1) 
                next_state = STATE_3; // 如果flag_2_to_3 == 1，转到STATE_3
            else 
                next_state = STATE_2; // 否则保持在STATE_2
        end
        STATE_3: begin
            if(flag_3_to_4 == 1) 
                next_state = STATE_4; // 如果flag_3_to_4 == 1，转到STATE_4
            else 
                next_state = STATE_3; // 否则保持在STATE_3
        end
        STATE_4: begin
            if(flag_4_to_5 == 1) 
                next_state = STATE_5; // 如果flag_4_to_5 == 1，转到STATE_5
            else 
                next_state = STATE_4; // 否则保持在STATE_4
        end        
        STATE_5: begin 
            if(flag_5_to_6 == 1) 
                next_state = STATE_6; // 如果flag_5_to_6 == 1，转到STATE_6
            else 
                next_state = STATE_5; // 否则保持在STATE_5
        end
        STATE_6: begin 
            if(flag_6_to_7 == 1) 
                next_state = STATE_7; // 如果flag_6_to_7 == 1，转到STATE_7
            else 
                next_state = STATE_6; // 否则保持在STATE_6
        end        
        STATE_7: begin
            if(flag_7_to_8 == 1) 
                next_state = STATE_8; // 如果flag_7_to_8 == 1，转到STATE_8
            else 
                next_state = STATE_7; // 否则保持在STATE_7
        end 
        STATE_8: begin
            if(flag_8_to_9 == 1) 
                next_state = STATE_9; // 如果flag_8_to_9 == 1，转到STATE_9
            else 
                next_state = STATE_8; // 否则保持在STATE_8
        end         
        STATE_9: begin
            if(flag_9_to_10 == 1) 
                next_state = STATE_10; // 如果flag_9_to_10 == 1，转到STATE_10
            else 
                next_state = STATE_9; // 否则保持在STATE_9
        end 
        STATE_10: begin
            if(flag_10_to_11 == 1) 
                next_state = STATE_11; // 如果flag_10_to_11 == 1，转到STATE_11
            else 
                next_state = STATE_10; // 否则保持在STATE_10
        end 
        STATE_11: begin
            if(flag_11_to_12 == 1) 
                next_state = STATE_12; // 如果flag_11_to_12 == 1，转到STATE_12
            else 
                next_state = STATE_11; // 否则保持在STATE_11
        end 
        STATE_12: begin
            if(flag_12_to_13 == 1) 
                next_state = STATE_13; // 如果flag_12_to_13 == 1，转到STATE_13
            else 
                next_state = STATE_12; // 否则保持在STATE_12
        end  
        STATE_13: begin
            if(flag_13_to_0 == 1) 
                next_state = STATE_0; // 如果flag_13_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_13; // 否则保持在STATE_13
        end        
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin 
        target_x <= 16'sd0;
        target_y <= 16'sd0;
        target_z <= -16'sd3800;
        
        delay_cnt <= 1'b0;
        sucker_pump <= 1'b0;
        sucker_valve <= 1'b0;
        ltem_cnt <= 0;
        ltem_cnt_flag <= 0;
        
        start_reset <= 1'b0;

        catch_finish <= 1'b0;

        pi_data <= 0;
        pi_flag <= 0;
        
        flag_0_to_1   <= 1'b0;
        flag_1_to_2   <= 1'b0; 
        flag_2_to_3   <= 1'b0; 
        flag_3_to_4   <= 1'b0;
        flag_4_to_5   <= 1'b0;
        flag_5_to_6   <= 1'b0;
        flag_6_to_7   <= 1'b0;
        flag_7_to_8   <= 1'b0;
        flag_8_to_9   <= 1'b0;
        flag_9_to_10  <= 1'b0;
        flag_10_to_11 <= 1'b0; 
        flag_11_to_12 <= 1'b0; 
        flag_12_to_13 <= 1'b0; 
        flag_13_to_0  <= 1'b0; 
    end
    else begin
        case(state)
            STATE_0: begin
            // 初始时刻运动到复位坐标,判断输入的信号是否有效 
                flag_13_to_0  <= 1'b0;
                
                ltem_cnt_flag <= 1'b0;
                sucker_pump   <= 1'b0;
                sucker_valve  <= 1'b0;
                pi_flag <= 0;
                
                if (data_valid) begin   // 数据有效时，分解输入的坐标
                    if (ltem_cnt == 3 && motion_done && Delta_start) begin   
                        start_reset <= 1'b1;   // 当抓取完成3个后，给出复位信号
                        catch_finish <= 1'b1;   // 表示三个物体抓完
                        
                        if (finish_reset == 1) begin
                            ltem_cnt <= 0;
                        end
                    end                    
                    
                    if (motion_done && Delta_start && (ltem_cnt < 3)) begin
                        catch_finish <= 1'b0;
                        
                        flag_0_to_1 <= 1'b1;
                    end
                    else begin
                        flag_0_to_1 <= 1'b0;                    
                    end
                end                    
            end
            STATE_1: begin  
            // 信号加一，抓取对应的物体
                flag_0_to_1 <= 1'b0; 
                
                start_reset <= 1'b0;
                
                if(!ltem_cnt_flag) begin
                    ltem_cnt <= ltem_cnt + 1;
                    ltem_cnt_flag <= 1;
                    flag_1_to_2 <= 1'b1;
                end
                else begin
                    flag_1_to_2 <= 1'b0;
                end
            end
            STATE_2: begin
            // 判断托盘编号
                flag_1_to_2 <= 1'b0;
                
                if (motion_done && Delta_start) begin
                    if(match_id == 1) begin
                        if(ltem_cnt == 1) begin
                            target_x <= target_item_take_x[ltem_red_1];
                            target_y <= target_item_take_y[ltem_red_1];
                            target_z <= target_item_take_z[ltem_red_1];
                            pi_data <= 8'h01;                             
                        end
                        else if(ltem_cnt == 2) begin
                            target_x <= target_item_take_x[ltem_red_2];
                            target_y <= target_item_take_y[ltem_red_2];
                            target_z <= target_item_take_z[ltem_red_2];
                            pi_data <= 8'h02; 
                        end
                        else begin
                            target_x <= target_item_take_x[ltem_red_3];
                            target_y <= target_item_take_y[ltem_red_3];
                            target_z <= target_item_take_z[ltem_red_3];
                            pi_data <= 8'h03; 
                        end
                    end
                    else if(match_id == 2) begin
                        if(ltem_cnt == 1) begin
                            target_x <= target_item_take_x[ltem_green_1];
                            target_y <= target_item_take_y[ltem_green_1];
                            target_z <= target_item_take_z[ltem_green_1];
                            pi_data <= 8'h04; 
                        end
                        else if(ltem_cnt == 2) begin
                            target_x <= target_item_take_x[ltem_green_2];
                            target_y <= target_item_take_y[ltem_green_2];
                            target_z <= target_item_take_z[ltem_green_2]; 
                            pi_data <= 8'h05;
                        end
                        else begin
                            target_x <= target_item_take_x[ltem_green_3];
                            target_y <= target_item_take_y[ltem_green_3];
                            target_z <= target_item_take_z[ltem_green_3];
                            pi_data <= 8'h06;
                        end
                    end
                    else if(match_id == 3) begin
                        if(ltem_cnt == 1) begin
                            target_x <= target_item_take_x[ltem_blue_1];
                            target_y <= target_item_take_y[ltem_blue_1];
                            target_z <= target_item_take_z[ltem_blue_1];
                            pi_data <= 8'h07; 
                        end
                        else if(ltem_cnt == 2) begin
                            target_x <= target_item_take_x[ltem_blue_2];
                            target_y <= target_item_take_y[ltem_blue_2];
                            target_z <= target_item_take_z[ltem_blue_2]; 
                            pi_data <= 8'h08;
                        end
                        else begin
                            target_x <= target_item_take_x[ltem_blue_3];
                            target_y <= target_item_take_y[ltem_blue_3];
                            target_z <= target_item_take_z[ltem_blue_3]; 
                            pi_data <= 8'h09;
                        end
                    end                    
                    else if(match_id == 4) begin
                        if(ltem_cnt == 1) begin
                            target_x <= target_item_take_x[ltem_yellow_1];
                            target_y <= target_item_take_y[ltem_yellow_1];
                            target_z <= target_item_take_z[ltem_yellow_1]; 
                            pi_data <= 8'h0A;
                        end
                        else if(ltem_cnt == 2) begin
                            target_x <= target_item_take_x[ltem_yellow_2];
                            target_y <= target_item_take_y[ltem_yellow_2];
                            target_z <= target_item_take_z[ltem_yellow_2]; 
                            pi_data <= 8'h0B;
                        end
                        else begin
                            target_x <= target_item_take_x[ltem_yellow_3];
                            target_y <= target_item_take_y[ltem_yellow_3];
                            target_z <= target_item_take_z[ltem_yellow_3]; 
                            pi_data <= 8'h0C;
                        end
                    end
                    
                    pi_flag <= 1;

                    flag_2_to_3 <= 1'b1; 
                end
                else if (!motion_done) begin
                    flag_2_to_3 <= 1'b0;                    
                end 
            end          
            STATE_3: begin 
            // 保证motion_done被拉低
                flag_2_to_3 <= 1'b0;     
                
                pi_flag <= 0;

                if (!motion_done && Delta_start) begin
                    flag_3_to_4 <= 1'b1;
                end
                else begin
                    flag_3_to_4 <= 1'b0;
               end
            end            
            STATE_4: begin
            // 抓取物体
                flag_3_to_4 <= 1'b0;

                if (motion_done && Delta_start) begin

                    if (delay_cnt < pump_stay_time) begin
                        delay_cnt <= delay_cnt + 1;
                        
                        flag_4_to_5 <= 1'b0;
                    end
                    else begin
                        delay_cnt <= 0;
                        
                        target_z <= item_take_z;
                        sucker_pump <= 1;

                        flag_4_to_5 <= 1'b1;                        
                    end                   
                end
                else if (!motion_done) begin
                    flag_4_to_5 <= 1'b0;                    
                end 
            end            
            STATE_5: begin             
            // 保证motion_done被拉低
                flag_4_to_5 <= 1'b0;    
                if (!motion_done && Delta_start) begin
                
                    flag_5_to_6 <= 1'b1;
                end
                else begin
                    flag_5_to_6 <= 1'b0;
               end
            end
            STATE_6: begin
            // 延迟等待，保证吸取
                flag_5_to_6 <= 1'b0;
                if (motion_done && Delta_start) begin
                    if (delay_cnt < pump_stay_time) begin
                        delay_cnt <= delay_cnt + 1;
                        
                        flag_6_to_7 <= 1'b0;
                    end
                    else begin
                        delay_cnt <= 0;

                        flag_6_to_7 <= 1'b1;                       
                    end
                end    
                else if (!motion_done) begin
                    flag_6_to_7 <= 1'b0;
                end
            end 
            STATE_7: begin
            // 垂直向上，吸取物体
                flag_6_to_7 <= 1'b0;
                
                target_z <= item_up_z; 

                flag_7_to_8 <= 1'b1;               
            end
            STATE_8: begin
            // 保证motion_done被拉低 
                flag_7_to_8 <= 1'b0; 
            
                if (!motion_done && Delta_start) begin

                    flag_8_to_9 <= 1'b1; 
                end
                else begin
                    flag_8_to_9 <= 1'b0;
                end
            end 
            STATE_9: begin
            // 运动到放置一号物体坐标
                flag_8_to_9 <= 1'b0;
                
                if(ltem_cnt == 1) begin
                    target_x <= target_item_put_x[0];
                    target_y <= target_item_put_y[0];
                    target_z <= target_item_put_z[0];  
                end
                else if(ltem_cnt == 2) begin
                    target_x <= target_item_put_x[1];
                    target_y <= target_item_put_y[1];
                    target_z <= target_item_put_z[1]; 
                end
                else begin
                    target_x <= target_item_put_x[2];
                    target_y <= target_item_put_y[2];
                    target_z <= target_item_put_z[2]; 
                end

                flag_9_to_10 <= 1'b1;                
            end           
            STATE_10: begin
            // 保证motion_done被拉低 

                flag_9_to_10 <= 1'b0;

                if (!motion_done && Delta_start) begin

                    flag_10_to_11 <= 1'b1;
                end
                else begin
                    flag_10_to_11 <= 1'b0;
                end
            end
            STATE_11: begin
            // 放置物体在托盘一号坐标上         
                flag_10_to_11 <= 1'b0;
                
                if(motion_done && Delta_start) begin 

                    if (delay_cnt < pump_put_time) begin
                        delay_cnt <= delay_cnt + 1;

                        if (delay_cnt == (pump_put_time >> 2)) begin
                        	sucker_pump <= 0;
                        end
                       
                        if (delay_cnt == (pump_put_time >> 1)) begin
                        	sucker_valve <= 1;
                        end
                        
                        flag_11_to_12 <= 1'b0;
                    end
                    else begin
                        delay_cnt <= 0;
                        
                        flag_11_to_12 <= 1'b1;                        
                    end
                end    
                else if (!motion_done) begin
                    flag_11_to_12 <= 1'b0;
                end                
            end
            STATE_12: begin
            // 运动到放置一号物体坐标
                flag_11_to_12 <= 1'b0;
                
                if(ltem_cnt == 3) begin
                    target_x <= 0;
                    target_y <= 0;
                    target_z <= -16'sd3800;  
                end

                flag_12_to_13 <= 1'b1;                
            end           
            STATE_13: begin
            // 保证motion_done被拉低 
                flag_12_to_13 <= 1'b0; 
                
                if(ltem_cnt == 3) begin
                    if(!motion_done && Delta_start) begin

                        flag_13_to_0 <= 1'b1;
                    end
                    else begin
                        flag_13_to_0 <= 1'b0;
                    end
                end
                else begin
                    flag_13_to_0 <= 1'b1;
                end
            end            
            default: ; 
        endcase
    end
end 



always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        s <= S_0;
    end else begin
        s <= next_s;
    end
end

always @(*) begin
    case (s)
        S_0:  next_s = f_0_to_1   ? S_1  : S_0;
        S_1:  next_s = f_1_to_2   ? S_2  : S_1;
        S_2:  next_s = f_2_to_3   ? S_3  : S_2;
        S_3:  next_s = f_3_to_4   ? S_4  : S_3;
        S_4:  next_s = f_4_to_5   ? S_5  : S_4;
        S_5:  next_s = f_5_to_6   ? S_6  : S_5;
        S_6:  next_s = f_6_to_7   ? S_7  : S_6;
        S_7:  next_s = f_7_to_8   ? S_8  : S_7;
        S_8:  next_s = f_8_to_9   ? S_9  : S_8;
        S_9:  next_s = f_9_to_10  ? S_10 : S_9;
        S_10: next_s = f_10_to_11 ? S_11 : S_10;
        S_11: next_s = f_11_to_12 ? S_12 : S_11;
        S_12: next_s = f_12_to_13 ? S_13 : S_12;
        S_13: next_s = f_13_to_14 ? S_14 : S_13;
        S_14: next_s = f_14_to_15 ? S_15 : S_14;
        S_15: next_s = f_15_to_16 ? S_16 : S_15;
        S_16: next_s = f_16_to_17 ? S_17 : S_16;
        S_17: next_s = f_17_to_18 ? S_18 : S_17;
        S_18: next_s = f_18_to_19 ? S_19 : S_18;
        S_19: next_s = f_19_to_20 ? S_20 : S_19;
        S_20: next_s = f_20_to_21 ? S_21 : S_20;
        S_21: next_s = f_21_to_22 ? S_22 : S_21;
        S_22: next_s = f_22_to_23 ? S_23 : S_22;
        S_23: next_s = f_23_to_24 ? S_24 : S_23;
        S_24: next_s = f_24_to_25 ? S_25 : S_24;
        S_25: next_s = f_25_to_26 ? S_26 : S_25;
        S_26: next_s = f_26_to_0  ? S_0  : S_26;
        default: next_s = S_0;
    endcase
end

//********************************************************************//
//****************** 平行四边形连杆机械臂的控制逻辑 ******************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin  
        arm_target_x <= 16'd0;
        arm_target_y <= 16'd0;
        arm_target_z <= 16'd0;
        
        arm_delay_cnt <= 0;
        stepper_motor_1_angles <= 0;
        turn_direction <= 1'b1;
        electromagnet <= 1'b0;

        plate_cnt <= 0;
        plate_cnt_flag <= 0;

        Delta_start <= 1'b0;
  
        match_id_pre <= 0;
        
        f_0_to_1   <= 1'b0;
        f_1_to_2   <= 1'b0;
        f_2_to_3   <= 1'b0;
        f_3_to_4   <= 1'b0;
        f_4_to_5   <= 1'b0;
        f_5_to_6   <= 1'b0;
        f_6_to_7   <= 1'b0;
        f_7_to_8   <= 1'b0;
        f_8_to_9   <= 1'b0;
        f_9_to_10  <= 1'b0;
        f_10_to_11 <= 1'b0;
        f_11_to_12 <= 1'b0;
        f_12_to_13 <= 1'b0;
        f_13_to_14 <= 1'b0;
        f_14_to_15 <= 1'b0;
        f_15_to_16 <= 1'b0;
        f_16_to_17 <= 1'b0;
        f_17_to_18 <= 1'b0;
        f_18_to_19 <= 1'b0;
        f_19_to_20 <= 1'b0;
        f_20_to_21 <= 1'b0;
        f_21_to_22 <= 1'b0;
        f_22_to_23 <= 1'b0;
        f_23_to_24 <= 1'b0;
        f_24_to_25 <= 1'b0;
        f_25_to_26 <= 1'b0;
        f_26_to_0  <= 1'b0;
    end
    else begin
        case(s)
            S_0: begin
                f_26_to_0 <= 0;
                
                if(start_move && arm_motion_done) begin
                    arm_target_x <= 16'd248;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
                    if(!plate_cnt_flag) begin
                        plate_cnt <= plate_cnt + 1;
                        plate_cnt_flag <= 1;
                    end
                    
                    f_0_to_1 <= 1;
                end
                else begin
                    f_0_to_1 <= 0;
                end
            end
            S_1: begin  
                f_0_to_1 <= 0;  
                
                if (!arm_motion_done) begin
                    plate_cnt_flag <= 0;
                    
                    f_1_to_2 <= 1;
                end
                else begin
                    f_1_to_2 <= 0;
                end
            end             
            S_2: begin
                f_1_to_2 <= 0;
                     
                if (arm_motion_done) begin
                
                    arm_target_x <= 16'd248;
                    arm_target_y <= 16'd0;

                    if(plate_cnt == 1) begin
                        arm_target_z <= -16'd45;
                    end
                    else if(plate_cnt == 2) begin
                        arm_target_z <= -16'd60;
                    end
                    else if(plate_cnt == 3) begin
                        arm_target_z <= -16'd75;
                    end
                    else if(plate_cnt == 4) begin
                        arm_target_z <= -16'd90;
                    end
                    
                    electromagnet <= 1;                    
                    
                    f_2_to_3 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_2_to_3 <= 0;                    
                end
            end 
            S_3: begin  
                f_2_to_3 <= 0;  
                
                if (!arm_motion_done) begin

                    f_3_to_4 <= 1;
                end
                else begin
                    f_3_to_4 <= 0;
                end
            end            
            S_4: begin
                f_3_to_4 <= 0;
                      
                if (arm_motion_done) begin
                    arm_target_x <= 16'd218;
                    arm_target_y <= 16'd0;
   
                    if(plate_cnt == 1) begin
                        arm_target_z <= -16'd15;
                    end
                    else if(plate_cnt == 2) begin
                        arm_target_z <= -16'd30;
                    end
                    else if(plate_cnt == 3) begin
                        arm_target_z <= -16'd45;
                    end
                    else if(plate_cnt == 4) begin
                        arm_target_z <= -16'd60;
                    end
                    
                    f_4_to_5 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_4_to_5 <= 0;                    
                end
            end 
            S_5: begin  
                f_4_to_5 <= 0;  
                
                if (!arm_motion_done) begin

                    f_5_to_6 <= 1;
                end
                else begin
                    f_5_to_6 <= 0;
                end
            end 
            S_6: begin
                f_5_to_6 <= 0;
                      
                if (arm_motion_done) begin
                    arm_target_x <= -16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd50;
                    
                    turn_direction <= 0;                    
                    
                    stepper_motor_1_angles <= 32000;
                    
                    f_6_to_7 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_6_to_7 <= 0;                    
                end
            end 
            S_7: begin  
                f_6_to_7 <= 0;  
                
                if (!arm_motion_done) begin

                    f_7_to_8 <= 1;
                end
                else begin
                    f_7_to_8 <= 0;
                end
            end  
            S_8: begin
                f_7_to_8 <= 0;
                      
                if (arm_motion_done && reached_target_1) begin
                    arm_target_x <= 16'd180;
                    arm_target_y <= 16'd5;
                    arm_target_z <= -16'd18;
                    
                    turn_direction <= 0;                    
                    
                    stepper_motor_1_angles <= 60416;
                    
                    f_8_to_9 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_8_to_9 <= 0;                    
                end
            end 
            S_9: begin  
                f_8_to_9 <= 0;  
                
                if (!arm_motion_done) begin
                    
                    f_9_to_10 <= 1;
                end
                else begin
                    f_9_to_10 <= 0;
                end
            end
            S_10: begin
                f_9_to_10 <= 0;
                      
                if (arm_motion_done && reached_target_1) begin
                    
                    f_10_to_11 <= 1;
                end
                else begin
                    f_10_to_11 <= 0;
                end
            end            
            S_11: begin
                f_10_to_11 <= 0;
                
                match_id_pre <= match_id;
                
                if(match_id_pre != match_id)begin
                    
                    match_id_pre <= match_id;
                    
                    arm_delay_cnt <= 0;
                    
                    f_11_to_12 <= 1;
                end
                else begin
                    f_11_to_12 <= 0;
                end
            end 
            S_12: begin
                f_11_to_12 <= 0; 
                
                Delta_start <= 1;   // 给出信号，表示Delta机械臂可以开始抓取
 
                if (arm_delay_cnt < 10) begin
                    arm_delay_cnt <= arm_delay_cnt + 1;
                       
                    f_12_to_13 <= 0;
                end
                else begin
                    arm_delay_cnt <= 0;
                    
                    f_12_to_13 <= 1;                        
                end
            end      
            S_13: begin
                f_12_to_13 <= 0;
                      
                if (arm_motion_done && catch_finish) begin
                    Delta_start <= 0;   // 关闭Delta机械臂 
                
                    arm_target_x <= -16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd50;
                    
                    turn_direction <= 1;                    
                    
                    stepper_motor_1_angles <= 32000;
                    
                    f_13_to_14 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_13_to_14 <= 0;                    
                end
            end  
            S_14: begin  
                f_13_to_14 <= 0;  
                
                if (!arm_motion_done) begin

                    f_14_to_15 <= 1;
                end
                else begin
                    f_14_to_15 <= 0;
                end
            end  
            S_15: begin
                f_14_to_15 <= 0;
                      
                if (arm_motion_done) begin
                    arm_target_x <= -16'd250;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd50;
                    
                    turn_direction <= 1;                    

                    if(match_id == 1) begin
                        stepper_motor_1_angles <= 0;
                    end
                    else if(match_id == 2) begin
                        stepper_motor_1_angles <= 30000;
                    end
                    else if(match_id == 3) begin
                        stepper_motor_1_angles <= 60000;
                    end
                    else if(match_id == 4) begin
                        stepper_motor_1_angles <= 90000;
                    end
                    
                    f_15_to_16 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_15_to_16 <= 0;                    
                end
            end
            S_16: begin  
                f_15_to_16 <= 0;  
                
                if (!arm_motion_done) begin

                    f_16_to_17 <= 1;
                end
                else begin
                    f_16_to_17 <= 0;
                end
            end
            S_17: begin
                f_16_to_17 <= 0;
                      
                if (arm_motion_done && reached_target_1) begin
                    arm_target_x <= -16'd250;
                    arm_target_y <= 16'd0;
                    arm_target_z <= -16'd100;

                    f_17_to_18 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_17_to_18 <= 0;                    
                end
            end 
            S_18: begin  
                f_17_to_18 <= 0;  
                
                if (!arm_motion_done) begin

                    f_18_to_19 <= 1;
                end
                else begin
                    f_18_to_19 <= 0;
                end
            end 
            S_19: begin        
                f_18_to_19 <= 0;
                
                if (arm_motion_done) begin   // 放置托盘

                    if (arm_delay_cnt < pump_put_time) begin
                        arm_delay_cnt <= arm_delay_cnt + 1;
                        
                        if (arm_delay_cnt == (pump_put_time >> 1)) begin
                            electromagnet <= 0; 
                        end
                        
                        f_19_to_20 <= 0;
                    end
                    else begin
                        arm_delay_cnt <= 0;
                        
                        f_19_to_20 <= 1;                        
                    end
                end    
                else if (!arm_motion_done) begin
                    f_19_to_20 <= 0;
                end                
            end
            S_20: begin
                f_19_to_20 <= 0;
                      
                if (arm_motion_done) begin
                    arm_target_x <= -16'd250;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
                    stepper_motor_1_angles <= 0;
                    
                    f_20_to_21 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_20_to_21 <= 0;                    
                end
            end 
            S_21: begin  
                f_20_to_21 <= 0;  
                
                if (!arm_motion_done) begin

                    f_21_to_22 <= 1;
                end
                else begin
                    f_21_to_22 <= 0;
                end
            end       
            S_22: begin
                f_21_to_22 <= 0;
                      
                if (arm_motion_done && reached_target_1) begin
                    arm_target_x <= -16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
                    f_22_to_23 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_22_to_23 <= 0;                    
                end
            end 
            S_23: begin  
                f_22_to_23 <= 0;  
                
                if (!arm_motion_done) begin

                    f_23_to_24 <= 1;
                end
                else begin
                    f_23_to_24 <= 0;
                end
            end           
            S_24: begin
                f_23_to_24 <= 0;
                      
                if (arm_motion_done) begin
                    arm_target_x <= 16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
                    turn_direction <= 1;
                    
                    f_24_to_25 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_24_to_25 <= 0;                    
                end
            end 
            S_25: begin  
                f_24_to_25 <= 0;  
                
                if (!arm_motion_done) begin

                    f_25_to_26 <= 1;
                end
                else begin
                    f_25_to_26 <= 0;
                end
            end            
            S_26: begin
                f_25_to_26 <= 0;
                      
                if (arm_motion_done && reached_target_1) begin
                    
                    f_26_to_0 <= 1;
                end            
                else if (!arm_motion_done) begin
                    f_26_to_0 <= 0;                    
                end
            end            
            default: ; 
        endcase
    end
end 

endmodule
