//三个主动臂角度同步解算，主动臂垂直向下为0°
//主动臂向外角度为正，范围0到140

module motion_solver
(
    input wire                  sys_clk,                // 系统时钟输入，50MHz频率    
    input wire                  sys_rst_n,              // 复位信号输入，低电平复位

    input wire signed [15:0]    x_16,                   // 放大16倍的x坐标     
    input wire signed [15:0]    y_16,                   // 放大16倍的y坐标 
    input wire signed [15:0]    z_16,                   // 放大16倍的z坐标

    output wire                 step_pulse_1,           // 步进电机的脉冲信号输出
    output wire                 direction_1,            // 步进电机的方向信号输出
    output wire                 reached_target_1,       // 目标位置标志位
    
    output wire                 step_pulse_2,           // 步进电机的脉冲信号输出
    output wire                 direction_2,            // 步进电机的方向信号输出
    output wire                 reached_target_2,       // 目标位置标志位
    
    output wire                 step_pulse_3,           // 步进电机的脉冲信号输出
    output wire                 direction_3,            // 步进电机的方向信号输出
    output wire                 reached_target_3,       // 目标位置标志位
    
    output reg                  motion_solver_work_done // 解算运动完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg  signed  [15:0] pretreat_data_x_16;     
reg  signed  [15:0] pretreat_data_y_16; 
reg  signed  [15:0] pretreat_data_z_16;
wire signed  [63:0] A_1;
wire signed  [63:0] B_1;
wire signed  [63:0] C_1;
wire signed  [63:0] A_2;
wire signed  [63:0] B_2;
wire signed  [63:0] C_2;
wire signed  [63:0] A_3;
wire signed  [63:0] B_3;
wire signed  [63:0] C_3;
wire                pretreat_data_work_done;

reg  signed  [63:0] calculate_angle_A;
reg  signed  [63:0] calculate_angle_B;
reg  signed  [63:0] calculate_angle_C;
wire         [31:0] calculate_angle;
wire                calculate_angle_work_done;

reg          [31:0] stepper_motor_angles_1;
reg          [31:0] stepper_motor_angles_2;
reg          [31:0] stepper_motor_angles_3;

reg          [31:0] target_angles_1;
reg          [31:0] target_angles_2;
reg          [31:0] target_angles_3;

reg  signed  [15:0] current_x = 0;
reg  signed  [15:0] current_y = 0;                 
reg  signed  [15:0] current_z = 0;

parameter STATE_0 = 3'b000;                           
parameter STATE_1 = 3'b001;                                                                           
parameter STATE_2 = 3'b010;                                                                           
parameter STATE_3 = 3'b011;                                                                           
parameter STATE_4 = 3'b100;                                                                           
parameter STATE_5 = 3'b101;                                                                           
parameter STATE_6 = 3'b110;                                                                           
parameter STATE_7 = 3'b111;                                                                           

reg          [2:0]  state, next_state; 
reg                 flag_0_to_1;
reg                 flag_1_to_2;
reg                 flag_2_to_3;
reg                 flag_3_to_4;
reg                 flag_4_to_5;
reg                 flag_5_to_6;
reg                 flag_6_to_7;
reg                 flag_7_to_0;
reg          [15:0] cnt;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

pretreat_data pretreat_data_inst(
    .sys_rst_n(sys_rst_n),
    .sys_clk(sys_clk),
    .x_16(pretreat_data_x_16),     
    .y_16(pretreat_data_y_16),     
    .z_16(pretreat_data_z_16),         
    .A_1(A_1),      
    .B_1(B_1),      
    .C_1(C_1),      
    .A_2(A_2),      
    .B_2(B_2),      
    .C_2(C_2),      
    .A_3(A_3),      
    .C_3(C_3),      
    .B_3(B_3),      
    .work_done(pretreat_data_work_done)
);

calculate_angle calculate_angle_inst(
    .sys_rst_n(sys_rst_n),
    .sys_clk(sys_clk),
    .A(calculate_angle_A),        
    .B(calculate_angle_B),        
    .C(calculate_angle_C),        
    .final_angle(calculate_angle),    
    .work_done(calculate_angle_work_done) 
);

stepper_motor_ctrl stepper_motor_ctrl_inst_1(
    .sys_rst_n(sys_rst_n),
    .sys_clk(sys_clk),        
    .target_angles(stepper_motor_angles_1),    
    .step_pulse(step_pulse_1),  
    .direction(direction_1),     
    .reached_target(reached_target_1)    
);

stepper_motor_ctrl stepper_motor_ctrl_inst_2(
    .sys_rst_n(sys_rst_n),
    .sys_clk(sys_clk),       
    .target_angles(stepper_motor_angles_2),    
    .step_pulse(step_pulse_2),  
    .direction(direction_2),     
    .reached_target(reached_target_2)     
);

stepper_motor_ctrl stepper_motor_ctrl_inst_3(
    .sys_rst_n(sys_rst_n),
    .sys_clk(sys_clk),        
    .target_angles(stepper_motor_angles_3),    
    .step_pulse(step_pulse_3),  
    .direction(direction_3),     
    .reached_target(reached_target_3)     
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
            if(flag_7_to_0 == 1)
                next_state = STATE_0; // 如果flag_7_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_7; // 否则保持在STATE_7
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        motion_solver_work_done <= 1;
        pretreat_data_x_16 <= 0;     
        pretreat_data_y_16 <= 0; 
        pretreat_data_z_16 <= 0;       

        calculate_angle_A <= 0;
        calculate_angle_B <= 0;
        calculate_angle_C <= 0;

        stepper_motor_angles_1 <= 0;
        stepper_motor_angles_2 <= 0;
        stepper_motor_angles_3 <= 0;

        target_angles_1 <= 0;
        target_angles_2 <= 0;
        target_angles_3 <= 0;
        
        current_x <= 0;
        current_y <= 0;                 
        current_z <= 0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_6 <= 0;
        flag_6_to_7 <= 0;
        flag_7_to_0 <= 0;
        cnt <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                flag_7_to_0 <= 0;
                
                if (x_16 != current_x || y_16 != current_y || z_16 != current_z) begin
                    motion_solver_work_done <= 0;
                    
                    pretreat_data_x_16 <= x_16;
                    pretreat_data_y_16 <= y_16;
                    pretreat_data_z_16 <= z_16;
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end 
            STATE_1: begin
                flag_0_to_1 <= 0;

                if(pretreat_data_work_done) begin
                    calculate_angle_A <= A_1;
                    calculate_angle_B <= B_1;
                    calculate_angle_C <= C_1;
                    
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end                        
            STATE_2: begin
                flag_1_to_2 <= 0;               
                
                if (calculate_angle_work_done) begin
                    target_angles_1 <= (calculate_angle > 33280) ? 33280 : ((calculate_angle < 0) ? 0 : calculate_angle);
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end
            end 
            STATE_3: begin
                flag_2_to_3 <= 0;
 
                calculate_angle_A <= A_2;
                calculate_angle_B <= B_2;
                calculate_angle_C <= C_2;
                
                cnt <= cnt + 1;
                if(cnt) begin
                    flag_3_to_4 <= 1;
                    cnt <= 0;
                end
                else begin
                    flag_3_to_4 <= 0;
                end
            end                
            STATE_4: begin 
                flag_3_to_4 <= 0;
            
                if (calculate_angle_work_done) begin
                    target_angles_2 <= (calculate_angle > 33280) ? 33280 : ((calculate_angle < 0) ? 0 : calculate_angle);
                    flag_4_to_5 <= 1;
                end
                else begin
                    flag_4_to_5 <= 0;
                end
            end 
            STATE_5: begin                    
                flag_4_to_5 <= 0;

                calculate_angle_A <= A_3;
                calculate_angle_B <= B_3;
                calculate_angle_C <= C_3;
                
                cnt <= cnt + 1;
                if(cnt == 1) begin
                    flag_5_to_6 <= 1;
                    cnt <= 0;
                end
                else begin
                    flag_5_to_6 <= 0;
                end
            end                  
            STATE_6: begin 
                flag_5_to_6 <= 0; 
                
                if (calculate_angle_work_done) begin
                    target_angles_3 <= (calculate_angle > 33280) ? 33280 : ((calculate_angle < 0) ? 0 : calculate_angle);
                    flag_6_to_7 <= 1;
                end
                else begin
                    flag_6_to_7 <= 0;
                end
            end                
            STATE_7: begin            
                flag_6_to_7 <= 0;
                
                stepper_motor_angles_1 <= target_angles_1;
                stepper_motor_angles_2 <= target_angles_2;
                stepper_motor_angles_3 <= target_angles_3;
                
                current_x <= x_16;
                current_y <= y_16;
                current_z <= z_16;
                
                if(cnt < 110) begin
                    cnt <= cnt + 1;
                end
                
                if(reached_target_1 && reached_target_2 && reached_target_3 && (cnt > 100)) begin 
                    
                    motion_solver_work_done <= 1;
                    flag_7_to_0 <= 1;
                end
                else begin
                    flag_7_to_0 <= 0;
                end
            end
            default: ; 
        endcase
    end
end

endmodule
