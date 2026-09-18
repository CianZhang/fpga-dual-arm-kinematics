module ram_ctrl
(
    input  wire  sys_clk,             // 系统时钟
    input  wire  sys_rst_n,           // 系统复位，低电平有效

    input  wire  start_inst,          // 启动初始化机械臂
    input  wire  start_move,          // 启动运动信号
    
    output wire  step_pulse_1,        // 平移步进电机脉冲信号
    output wire  direction_1,         // 平移步进电机方向信号
    output wire  reached_target_1,    // 平移达到目标位置标志位
            
    output wire  step_pulse_2,        // 旋转步进电机脉冲信号
    output wire  direction_2,         // 旋转步进电机方向信号
    output wire  reached_target_2,    // 旋转达到目标位置标志位    
            
    output wire  step_pulse_3,        // 大臂步进电机脉冲信号
    output wire  direction_3,         // 大臂步进电机方向信号
    output wire  reached_target_3,    // 大臂达到目标位置标志位
            
    output wire  step_pulse_4,        // 小臂步进电机脉冲信号
    output wire  direction_4,         // 小臂步进电机方向信号
    output wire  reached_target_4,    // 小臂达到目标位置标志位
    
    output reg   electromagnet,       // 电磁铁打开信号，高电平有效

    input  wire  RFID_rx,             // RFID读取标签rx
    output wire  RFID_tx,             // RFID读取标签tx
    
    output reg   work_done            // 完成吸取，高电平可以开始下一个 
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************// 

parameter pump_put_time    = 400;          // 吸盘停留时间4000000 

reg  signed [15:0] arm_target_x;
reg  signed [15:0] arm_target_y;
reg  signed [15:0] arm_target_z;
reg         [31:0] arm_delay_cnt;

reg                Delta_start;

reg         [3:0]  plate_cnt;
reg                plate_cnt_flag;

reg                get_plate_num;
wire        [2:0]  plate_num;
wire               plate_num_valid;

reg                turn_direction; 
reg         [31:0] stepper_motor_1_angles;
wire               arm_motion_done;

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

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

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

RFID_uart RFID_uart_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n), 
    .send_trig(get_plate_num),
    .rx(RFID_rx),       
    .tx(RFID_tx),       
    .match_id(plate_num), 
    .data_valid(plate_num_valid)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

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
        
        get_plate_num <= 1'b0;
        Delta_start <= 1'b0;
        
        work_done <= 1'b1;
 
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
                    
                    work_done <= 1'b0;
                    
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
                    else begin
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
                    else begin
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
                    arm_target_z <= 16'd0;
                    
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
                    arm_target_z <= -16'd16;
                    
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
                      
                if (arm_motion_done) begin
                    get_plate_num <= 1;
                        
                    f_10_to_11 <= 1;
                end    
                else if (!arm_motion_done) begin
                    f_10_to_11 <= 0;                    
                end
            end            
            S_11: begin
                f_10_to_11 <= 0;
                
                get_plate_num <= 0;
                
                if(!plate_num_valid) begin
                
                    f_11_to_12 <= 1;
                end
                else begin
                    f_11_to_12 <= 0;
                end
            end
            S_12: begin
                f_11_to_12 <= 0;            

                if (plate_num_valid) begin
                    Delta_start <= 1;   // 给出信号，表示Delta机械臂可以开始抓取             

                    f_12_to_13 <= 1;
                end
                else begin
                    f_12_to_13 <= 0;                    
                end
            end      
            S_13: begin
                f_12_to_13 <= 0;
                      
                //if (arm_motion_done && catch_finish) begin
                if (arm_motion_done) begin
                    Delta_start <= 0;   // 关闭Delta机械臂 
                
                    arm_target_x <= -16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
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
                    arm_target_z <= 16'd0;
                    
                    turn_direction <= 1;                    

                    if(plate_cnt == 1) begin
                        stepper_motor_1_angles <= 0;
                    end
                    else if(plate_cnt == 2) begin
                        stepper_motor_1_angles <= 29900;
                    end
                    else if(plate_cnt == 3) begin
                        stepper_motor_1_angles <= 59800;
                    end
                    else begin
                        stepper_motor_1_angles <= 89700;
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
                      
                if (arm_motion_done) begin
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
                      
                if (arm_motion_done) begin
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
                      
                if (arm_motion_done && reached_target_1) begin
                    arm_target_x <= 16'd180;
                    arm_target_y <= 16'd0;
                    arm_target_z <= 16'd0;
                    
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
                      
                if (arm_motion_done) begin
                    
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
