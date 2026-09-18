// 旋转角度的计算
// 输入空间坐标X和Y，计算出角度
// 角度以Y轴正半轴为0轴，顺时针为正，逆时针为负
//turn_direction 0顺 1逆

module solver_rotation
(
    input wire                  sys_clk,      
    input wire                  sys_rst_n, 

    input wire signed [15:0]    i_x,                    // 空间坐标X     
    input wire signed [15:0]    i_y,                    // 空间坐标Y
    
    input wire                  turn_direction,         // 旋转方向
    
    output wire                 step_pulse,             // 步进电机的脉冲信号输出
    output wire                 direction,              // 步进电机的方向信号输出
    output wire                 reached_target          // 目标位置标志位
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg  signed [31:0] cordic_x;
reg  signed [31:0] cordic_y;
wire        [31:0] cordic_output;
wire               cordic_valid;

reg  signed [31:0] raw_angle;
reg  signed [31:0] final_angle;

reg  signed [15:0] current_x;
reg  signed [15:0] current_y;  

parameter STATE_0 = 2'b00;                           
parameter STATE_1 = 2'b01;                
parameter STATE_2 = 2'b10;                      
parameter STATE_3 = 2'b11; 

reg         [1:0]  state, next_state;
reg                flag_0_to_1;
reg                flag_1_to_2;
reg                flag_2_to_3;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

cordic_serial_32 cordic_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x),     
    .y(cordic_y),   
    .phase(cordic_output),
    .valid(cordic_valid)
);

stepper_motor_ctrl_2 stepper_motor_ctrl_2_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle),
    .turn_direction(turn_direction),
    .step_pulse(step_pulse),
    .direction(direction),
    .reached_target(reached_target)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= STATE_0;            
    end else begin
        state <= next_state;          
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
            next_state = STATE_0;
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end



always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cordic_x <= 0;
        cordic_y <= 0;
        
        current_x <= 0;
        current_y <= 0;
        
        raw_angle <= 0;
        final_angle <= 0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                
                if (i_x != current_x || i_y != current_y) begin
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end    
            STATE_1: begin 
                flag_0_to_1 <= 0;
                cordic_x <= i_x;
                cordic_y <= i_y;
                if (cordic_valid == 0) begin 
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end    
            STATE_2: begin
                flag_1_to_2 <= 0; 
                if (cordic_valid) begin
                    raw_angle <= ((cordic_output >> 8) - 1500);
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end
            end
            STATE_3: begin
                flag_2_to_3 <= 0;
                if (i_x > 0 && i_y == 0) begin
                    final_angle <= 21540;            // 23040 - 1500
                end
                else if (i_x == 0 && i_y > 0) begin
                    final_angle <= 44580;			 // 46080 - 1500
                end
                else if (i_x < 0 && i_y == 0) begin
                    final_angle <= 67620;            // 69120 - 1500
                end 
                else if (i_x == 0 && i_y < 0) begin
                    final_angle <= -1500;             // 0 - 1500
                end
                else if (i_x > 0 && i_y < 0) begin
                    final_angle <= raw_angle - 69120;
                end
                else begin
                    final_angle <= raw_angle + 23040;
                end                 
            end   
            default: ; 
        endcase
    end
end 

endmodule
