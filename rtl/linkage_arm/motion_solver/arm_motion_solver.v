// 逆运动学解算顶层文件
// 输入原始的空间坐标
// 分时复用，只使用一个乘法器

module arm_motion_solver
(
    input wire                  sys_clk,      
    input wire                  sys_rst_n, 

    input wire signed [15:0]    i_x,                    // 空间坐标X     
    input wire signed [15:0]    i_y,                    // 空间坐标Y
    input wire signed [15:0]    i_z,                    // 空间坐标Z
    input wire                  turn_direction,         // 旋转轴旋转方向
    
    output wire                 step_pulse_2,           // 步进电机的脉冲信号输出
    output wire                 direction_2,            // 步进电机的方向信号输出
    output wire                 reached_target_2,       // 目标位置标志位
    output wire                 step_pulse_3,           // 步进电机的脉冲信号输出
    output wire                 direction_3,            // 步进电机的方向信号输出
    output wire                 reached_target_3,       // 目标位置标志位
    output wire                 step_pulse_4,           // 步进电机的脉冲信号输出
    output wire                 direction_4,            // 步进电机的方向信号输出
    output wire                 reached_target_4,       // 目标位置标志位
    
    output reg                  work_done               //解算运动完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg                     ip_multiplier_work;            // 乘法器工作信号，高电平有效
wire signed      [63:0] p;                             // 边长平方       
reg  signed      [31:0] a;           
reg  signed      [31:0] b;

reg  signed      [63:0] p_x;                           // 边长平方
reg  signed      [63:0] p_y;                           // 边长平方
reg  signed      [63:0] p_tran;                        // 边长平方

reg              [31:0] data_x_34;                     // 开方结果为i_x_34 
wire             [15:0] sqrt_x_34;                        
wire                    sqrt_valid_x_34;

reg  signed      [15:0] i_x_2;                         // 用于旋转角度的计算，真实值X  
reg  signed      [15:0] i_y_2;                         // 用于旋转角度的计算，真实值Y

reg  signed      [15:0] i_x_34;                        // 用于大臂小臂的计算，长度 
reg  signed      [15:0] i_y_34;                        // 用于大臂小臂的计算，真实值Z

reg  signed      [15:0] current_x = 0;
reg  signed      [15:0] current_y = 0;                 
reg  signed      [15:0] current_z = 0;

reg              [31:0] delay_counter;

parameter STATE_0 = 3'd0;                           
parameter STATE_1 = 3'd1;                
parameter STATE_2 = 3'd2;                      
parameter STATE_3 = 3'd3;
parameter STATE_4 = 3'd4;
parameter STATE_5 = 3'd5;
parameter STATE_6 = 3'd6;

reg              [2:0]  state, next_state;
reg                     flag_0_to_1;
reg                     flag_1_to_2;
reg                     flag_2_to_3;
reg                     flag_3_to_4;
reg                     flag_4_to_5;
reg                     flag_5_to_6;
reg                     flag_6_to_0;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

ip_multiplier ip_multiplier_inst (
    .ce(ip_multiplier_work),
    .rst(~sys_rst_n),
    .clk(sys_clk),
    .a(a),
    .b(b),
    .p(p)
);

sqrt_32 sqrt_x_34_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_x_34),        
    .sqrt(sqrt_x_34),     
    .valid(sqrt_valid_x_34)     
);

solver_rotation solver_rotation_inst (
    .sys_clk(sys_clk),      
    .sys_rst_n(sys_rst_n), 
    .i_x(i_x_2),                     
    .i_y(i_y_2), 
    .turn_direction(turn_direction),    
    .step_pulse(step_pulse_2),           
    .direction(direction_2),            
    .reached_target(reached_target_2)      
);

solver_big_and_small_arm solver_big_and_small_arm_inst (
    .sys_clk(sys_clk),      
    .sys_rst_n(sys_rst_n), 
    .i_x(i_x_34),                     
    .i_y(i_y_34),                    
    .step_pulse_3(step_pulse_3),         
    .direction_3(direction_3),          
    .reached_target_3(reached_target_3),      
    .step_pulse_4(step_pulse_4),         
    .direction_4(direction_4),         
    .reached_target_4(reached_target_4)       
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
            if(flag_6_to_0 == 1) 
                next_state = STATE_0; // 如果flag_6_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_6; // 否则保持在STATE_6
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        current_x <= 0;
        current_y <= 0;                 
        current_z <= 0;
        
        ip_multiplier_work <= 0;
        a <= 0;           
        b <= 0;  
        
        p_x <= 0; 
        p_y <= 0;            
        p_tran <= 0;
        
        data_x_34 <= 0;
        
        i_x_2 <= 0;
        i_y_2 <= 0;
        i_x_34 <= 0;
        i_y_34 <= 0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_6 <= 0;
        flag_6_to_0 <= 0;
        
        work_done <= 1;
    end
    else begin
        case(state)
            STATE_0: begin              //对输入的值判断，是否更新
                delay_counter <= 0;
                flag_6_to_0 <= 0;
                
                data_x_34 <= 0;
                                    
                if (reached_target_2 && reached_target_3 && reached_target_4) begin
                    work_done <= 1;
                end
                else begin  
                    work_done <= 0;
                end

                if (i_x != current_x || i_y != current_y || i_z != current_z) begin
                    work_done <= 0;
                    
                    ip_multiplier_work <= 1;
                    a <= i_x;         
                    b <= i_x;                    

                    flag_0_to_1 <= 1;                    
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin 
                flag_0_to_1 <= 0;
                
                ip_multiplier_work <= 0;
                p_x <= p;

                flag_1_to_2 <= 1;
            end
            STATE_2: begin
                flag_1_to_2 <= 0;
            
                ip_multiplier_work <= 1;
                a <= i_y;         
                b <= i_y; 
                
                flag_2_to_3 <= 1;
            end
            STATE_3: begin   
                flag_2_to_3 <= 0;
                
                ip_multiplier_work <= 0;
                p_y <= p;

                flag_3_to_4 <= 1;
            end
            STATE_4: begin 
                flag_3_to_4 <= 0;

                data_x_34 <= p_x + p_y;

                if (sqrt_valid_x_34 == 0) begin
                    flag_4_to_5 <= 1;
                end
                else begin
                    flag_4_to_5 <= 0;
                end  
            end
            STATE_5: begin 
                flag_4_to_5 <= 0;              
                
                if (sqrt_valid_x_34 == 1) begin
                    i_x_2 <= i_x;
                    i_y_2 <= i_y;
                    
                    i_x_34 <= sqrt_x_34;
                    i_y_34 <= i_z;

                    current_x <= i_x;
                    current_y <= i_y;
                    current_z <= i_z;
                    
                    flag_5_to_6 <= 1;
                end
                else begin
                    flag_5_to_6 <= 0;
                end  
            end
            STATE_6: begin 
                flag_5_to_6 <= 0;
                
                if (delay_counter < 100) begin
                    delay_counter <= delay_counter + 1;
                    flag_6_to_0 <= 0;
                end
                else begin
                    flag_6_to_0 <= 1; 
                end
            end
            default: ; 
        endcase
    end
end   

endmodule    
