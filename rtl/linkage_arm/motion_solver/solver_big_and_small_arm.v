//大臂角度的计算和小臂角度的计算
//小臂步进电机转动的角度与大臂角有关
//输入变换后的坐标X，Y计算出角度，其中X = 真正的 根号下(X^2 + Y^2)，Y = 真正的 Z
//大臂与X轴负半轴的夹角，顺时针为正
//大臂与小臂的夹角，逆时针为正

module solver_big_and_small_arm
(
    input wire                  sys_clk,      
    input wire                  sys_rst_n, 

    input wire signed [15:0]    i_x,                    // 计算之后的X     
    input wire signed [15:0]    i_y,                    // 计算之后的Y
  
    output wire                 step_pulse_3,           // 步进电机的脉冲信号输出
    output wire                 direction_3,            // 步进电机的方向信号输出
    output wire                 reached_target_3,       // 目标位置标志位
    
    output wire                 step_pulse_4,           // 步进电机的脉冲信号输出
    output wire                 direction_4,            // 步进电机的方向信号输出
    output wire                 reached_target_4        // 目标位置标志位
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam start_elevation  = 95;                        // 起始升高量
localparam big_arm_length   = 150;                       // 大臂长
localparam small_arm_length = 150;                       // 小臂长
localparam end_extension    = 65;                        // 末端前伸量
localparam end_descent      = 58;                        // 末端下降量

localparam big_arm_length_2   = 22500;                   // 大臂长的平方
localparam small_arm_length_2 = 22500;                   // 小臂长的平方
localparam small_and_big_2    = 45000;                   // 2 * 大臂长 * 小臂长

reg                     ip_multiplier_work;              // 乘法器工作信号，高电平有效
wire signed      [63:0] p;                               // 边长平方       
reg  signed      [31:0] a;           
reg  signed      [31:0] b;

reg  signed      [63:0] p_x;                             // 边长平方
reg  signed      [63:0] p_y;                             // 边长平方
reg  signed      [63:0] p_judge;                         // 用于判断大臂角是否大于180
reg  signed      [63:0] p_cos;                           // 小臂角cos的平方
reg  signed      [63:0] p_sina_sinb_numerator;           // sina_sinb的分子相乘               
reg  signed      [63:0] p_sina_sinb_denominator;         // sina_sinb的分母相乘       
reg  signed      [63:0] p_cos_theta;                     // 对大臂角平方
reg  signed      [31:0] a_cos_theta;
  
reg              [31:0] data_length;                     // 对边长开方
wire             [15:0] sqrt_length;
wire                    sqrt_valid_length;
                 
reg              [31:0] data_sina;                       // 对sina的分子开方
wire             [15:0] sqrt_sina;                        
wire                    sqrt_valid_sina;                  
           
reg              [31:0] denominator_cosa_cosb;           // 分母
reg              [31:0] numerator_cosa_cosb;             // 分子     
reg                     start_cosa_cosb;                 // 启动信号
wire                    done_cosa_cosb;                  // 完成标志
wire             [31:0] quotient_cosa_cosb;              // 商
wire             [31:0] remainder_cosa_cosb;             // 余数   
                                                         
reg              [31:0] denominator_sina_sinb;           // 分母
reg              [31:0] numerator_sina_sinb;             // 分子     
reg                     start_sina_sinb;                 // 启动信号
wire                    done_sina_sinb;                  // 完成标志
wire             [31:0] quotient_sina_sinb;              // 商
wire             [31:0] remainder_sina_sinb;             // 余数

reg              [31:0] denominator;                     // 分母
reg              [31:0] numerator;                       // 分子     
reg                     start;                           // 启动信号
wire                    done;                            // 完成标志
wire             [31:0] quotient;                        // 商
wire             [31:0] remainder;                       // 余数

reg              [31:0] data_theta;                      // 对1 - 大臂角开方
wire             [15:0] sqrt_theta;
wire                    sqrt_valid_theta;

reg              [31:0] data;                            // 用于小臂角sin的开方
wire             [15:0] sqrt;
wire                    sqrt_valid;
         
reg  signed      [31:0] cordic_x_3;
reg  signed      [31:0] cordic_y_3;
wire             [31:0] cordic_output_3;
wire                    cordic_valid_3;
                 
reg signed       [31:0] raw_angle_3;
reg signed       [31:0] final_angle_3;

reg signed       [31:0] cordic_x_4;
reg signed       [31:0] cordic_y_4;
wire             [31:0] cordic_output_4;
wire                    cordic_valid_4;

reg  signed      [15:0] current_y = 0;                 
reg  signed      [15:0] current_x = 0; 

reg signed       [31:0] raw_angle_4;
reg signed       [31:0] final_angle_4;

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
parameter STATE_14 = 5'd14;
parameter STATE_15 = 5'd15;
parameter STATE_16 = 5'd16;
parameter STATE_17 = 5'd17;
parameter STATE_18 = 5'd18;
parameter STATE_19 = 5'd19;
parameter STATE_20 = 5'd20;
parameter STATE_21 = 5'd21;
parameter STATE_22 = 5'd22;

reg          [4:0] state, next_state;
reg                flag_0_to_1;
reg                flag_7_to_8;
reg                flag_8_to_9;
reg                flag_14_to_15;
reg                flag_19_to_20;
reg                flag_20_to_21;
reg                flag_21_to_22;

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

sqrt_32 sqrt_length_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_length),        
    .sqrt(sqrt_length),     
    .valid(sqrt_valid_length)     
);

sqrt_32 sqrt_sina_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_sina),        
    .sqrt(sqrt_sina),     
    .valid(sqrt_valid_sina)     
);

signed_divider signed_divider_cosa_cosb_inst (
    .clk(sys_clk),
    .rst(~sys_rst_n),
    .start(start_cosa_cosb),
    .dividend(numerator_cosa_cosb),
    .divisor(denominator_cosa_cosb),
    .quotient(quotient_cosa_cosb),
    .remainder(remainder_cosa_cosb),
    .zeroErr(),
    .valid(done_cosa_cosb),
    .overflow()
);     

signed_divider signed_divider_sina_sinb_inst (
    .clk(sys_clk),
    .rst(~sys_rst_n),
    .start(start_sina_sinb),
    .dividend(numerator_sina_sinb),
    .divisor(denominator_sina_sinb),
    .quotient(quotient_sina_sinb),
    .remainder(remainder_sina_sinb),
    .zeroErr(),
    .valid(done_sina_sinb),
    .overflow()
);

signed_divider signed_divider_inst (
    .clk(sys_clk),
    .rst(~sys_rst_n),
    .start(start),
    .dividend(numerator),
    .divisor(denominator),
    .quotient(quotient),
    .remainder(remainder),
    .zeroErr(),
    .valid(done),
    .overflow()
);

sqrt_32 sqrt_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data),        
    .sqrt(sqrt),     
    .valid(sqrt_valid)     
);

sqrt_32 sqrt_theta_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_theta),        
    .sqrt(sqrt_theta),     
    .valid(sqrt_valid_theta)     
);

cordic_serial_32 cordic_3_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x_3),     
    .y(cordic_y_3),   
    .phase(cordic_output_3),
    .valid(cordic_valid_3)
);

cordic_serial_32 cordic_4_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x_4),     
    .y(cordic_y_4),   
    .phase(cordic_output_4),
    .valid(cordic_valid_4)
);

stepper_motor_ctrl_3 stepper_motor_ctrl_3_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_3),
    .step_pulse(step_pulse_3),
    .direction(direction_3),
    .reached_target(reached_target_3)
);

stepper_motor_ctrl_4 stepper_motor_ctrl_4_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle_4),
    .step_pulse(step_pulse_4),
    .direction(direction_4),
    .reached_target(reached_target_4)
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
        STATE_0:  next_state = flag_0_to_1 ? STATE_1 : STATE_0;
        STATE_1:  next_state = STATE_2;
        STATE_2:  next_state = STATE_3;
        STATE_3:  next_state = STATE_4;
        STATE_4:  next_state = STATE_5;
        STATE_5:  next_state = STATE_6;
        STATE_6:  next_state = STATE_7;
        STATE_7: begin
            if(flag_7_to_8 == 1'b1) 
                next_state = STATE_8; // 如果flag_7_to_8 == 1，转到STATE_8
            else 
                next_state = STATE_7; // 否则保持在STATE_7
        end       
        STATE_8: begin
            if(flag_8_to_9 == 1'b1) 
                next_state = STATE_9; // 如果flag_8_to_9 == 1，转到STATE_9
            else 
                next_state = STATE_8; // 否则保持在STATE_8
        end         
        STATE_9:  next_state = STATE_10;
        STATE_10: next_state = STATE_11;
        STATE_11: next_state = STATE_12;        
        STATE_12: next_state = STATE_13;        
        STATE_13: begin
            if(done_sina_sinb == 1'b0) 
                next_state = STATE_14; // 如果done_sina_sinb == 0，转到STATE_14
            else 
                next_state = STATE_13; // 否则保持在STATE_13
        end        
        STATE_14: begin
            if(flag_14_to_15 == 1) 
                next_state = STATE_15; // 如果flag_14_to_15 = 1，转到STATE_15
            else 
                next_state = STATE_14; // 否则保持在STATE_14
        end
        STATE_15: next_state = STATE_16;
        STATE_16: next_state = STATE_17;          
        STATE_17: next_state = STATE_18;          
        STATE_18: next_state = STATE_19;          
        STATE_19: begin
            if(flag_19_to_20 == 1'b1) 
                next_state = STATE_20; // 如果flag_19_to_20 == 1，转到STATE_20
            else 
                next_state = STATE_19; // 否则保持在STATE_19
        end        
        STATE_20: begin
            if(flag_20_to_21 == 1'b1) 
                next_state = STATE_21; // 如果flag_20_to_21 == 1，转到STATE_21
            else 
                next_state = STATE_20; // 否则保持在STATE_20
        end        
        STATE_21: begin
            if(flag_21_to_22 == 1'b1) 
                next_state = STATE_22; // 如果flag_21_to_22 == 1，转到STATE_22
            else 
                next_state = STATE_21; // 否则保持在STATE_21
        end        
        STATE_22: next_state = STATE_0;        
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cordic_x_3 <= 0;
        cordic_y_3 <= 0;
        cordic_x_4 <= 0;
        cordic_y_4 <= 0;
        raw_angle_3 <= 0;
        final_angle_3 <= 0;
        raw_angle_4 <= 0;
        final_angle_4 <= 0;
        current_x <= 0;
        current_y <= 0;
        
        ip_multiplier_work <= 0; 
        a <= 0;
        b <= 0;
        
        p_x <= 0;           
        p_y <= 0;           
        p_cos <= 0;           
        p_cos_theta <= 0;           
        p_sina_sinb_numerator <= 0;                  
        p_sina_sinb_denominator <= 0;           
        p_judge <= 0;
        
        data_length <= 0;
        data_sina <= 0;
        data_theta <= 0;
        data <= 0;
        
        denominator_cosa_cosb <= 0;
        numerator_cosa_cosb <= 0; 
        start_cosa_cosb <= 0;  
        denominator_sina_sinb <= 0;
        numerator_sina_sinb <= 0; 
        start_sina_sinb <= 0;  
        denominator <= 0;
        numerator <= 0;     
        start <= 0; 

        flag_0_to_1 <= 0;
        flag_7_to_8 <= 0;
        flag_8_to_9 <= 0;
        flag_14_to_15 <= 0;
        flag_19_to_20 <= 0;
        flag_20_to_21 <= 0;
        flag_21_to_22 <= 0;      
    end
    else begin
        case(state)
            STATE_0: begin 
                start_cosa_cosb <= 0; 
                start_sina_sinb <= 0;  
                start <= 0;

                data_length <= 0;
                data_sina <= 0;
                data_theta <= 0;
                data <= 0; 
                    
                if(i_x != current_x || i_y != current_y) begin
                    current_x <= i_x;
                    current_y <= i_y;
                    
                    start_cosa_cosb <= 1; 
                    numerator_cosa_cosb <= ((i_x - end_extension) << 8);
                    denominator_cosa_cosb <= 300;   //2 * big_arm_length
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin
                ip_multiplier_work <= 1;
                if (i_y > (start_elevation - end_descent)) begin
                    a <= (i_x - end_extension);         
                    b <= (i_x - end_extension);
                end
                else if (i_y < (start_elevation - end_descent)) begin
                    a <= (i_x - end_extension);         
                    b <= (i_x - end_extension);
                end
                else begin
                    a <= (i_x - end_extension);         
                    b <= (i_x - end_extension);
                end
            end
            STATE_2: begin            
                ip_multiplier_work <= 0;
                p_x <= p;
            end
            STATE_3: begin
                ip_multiplier_work <= 1;
                if (i_y > (start_elevation - end_descent)) begin
                    a <= (i_y + end_descent - start_elevation);         
                    b <= (i_y + end_descent - start_elevation);
                end
                else if (i_y < (start_elevation - end_descent)) begin
                    a <= (start_elevation - i_y - end_descent);         
                    b <= (start_elevation - i_y - end_descent);
                end
                else begin
                    a <= 0;         
                    b <= 0;
                end
            end
            STATE_4: begin            
                ip_multiplier_work <= 0;
                p_y <= p;
            end         
            STATE_5: begin
                ip_multiplier_work <= 1;
                a <= (i_x - end_extension);         
                b <= 300;                
            end
            STATE_6: begin            
                ip_multiplier_work <= 0;
                p_judge <= p;
            end  
            STATE_7: begin
                flag_0_to_1 <= 0;
                
                start_cosa_cosb <= 0;
                
                data_length <= p_x + p_y;
                data_sina <= 90000 - p_x - p_y;     //4 * big_arm_length_2
                
                start <= 1;
                numerator <= ((big_arm_length_2 + small_arm_length_2 - p_x - p_y) << 8);
                denominator <= small_and_big_2;
                
                if(sqrt_valid_length == 0 && sqrt_valid_sina == 0) begin      
                    flag_7_to_8 <= 1;
                end
                else begin
                    flag_7_to_8 <= 0;
                end
            end
            STATE_8: begin
                flag_7_to_8 <= 0;
                
                start <= 0;
                
                if(sqrt_valid_length == 1 && sqrt_valid_sina == 1) begin 

                    flag_8_to_9 <= 1;
                end
                else begin
                    flag_8_to_9 <= 0;
                end
            end              
            STATE_9: begin                
                ip_multiplier_work <= 1;
                if (i_y > (start_elevation - end_descent)) begin
                    a <= sqrt_sina;         
                    b <= (i_y + end_descent - start_elevation); 
                end
                else if (i_y < (start_elevation - end_descent)) begin
                    a <= sqrt_sina;         
                    b <= (start_elevation - i_y - end_descent);
                end
                else begin
                    a <= 0;         
                    b <= 0;
                end
            end                
            STATE_10: begin            
                ip_multiplier_work <= 0;
                p_sina_sinb_numerator <= p;
            end                 
            STATE_11: begin                
                ip_multiplier_work <= 1;
                a <= 300;   //2 * big_arm_length          
                b <= sqrt_length; 
            end                
            STATE_12: begin            
                ip_multiplier_work <= 0;
                p_sina_sinb_denominator <= p;
            end 
            STATE_13: begin 
                flag_8_to_9 <= 0;
                
                start_sina_sinb <= 1;
                
                numerator_sina_sinb <= (p_sina_sinb_numerator << 8);  
                denominator_sina_sinb <= p_sina_sinb_denominator;
            end                  
            STATE_14: begin 
                start_sina_sinb <= 0;
                
                if (done_sina_sinb == 1) begin
                    
                    flag_14_to_15 <= 1;
                end
                else begin
                    flag_14_to_15 <= 0;
                end
            end
            STATE_15: begin                         
                ip_multiplier_work <= 1;
                a <= quotient;         
                b <= quotient; 
            end                
            STATE_16: begin            
                ip_multiplier_work <= 0;
                p_cos <= p;
            end
            STATE_17: begin                         
                ip_multiplier_work <= 1;
                if (i_y > (start_elevation - end_descent)) begin
                    a_cos_theta <= quotient_cosa_cosb - quotient_sina_sinb;
                    a <= quotient_cosa_cosb - quotient_sina_sinb;
                    b <= quotient_cosa_cosb - quotient_sina_sinb;
                end
                else if (i_y < (start_elevation - end_descent)) begin
                    a_cos_theta <= quotient_cosa_cosb + quotient_sina_sinb;
                    a <= quotient_cosa_cosb + quotient_sina_sinb;
                    b <= quotient_cosa_cosb + quotient_sina_sinb;
                end
                else begin
                    a_cos_theta <= quotient_cosa_cosb;
                    a <= quotient_cosa_cosb;
                    b <= quotient_cosa_cosb;
                end  
            end                
            STATE_18: begin            
                ip_multiplier_work <= 0;
                p_cos_theta <= p;
            end
            STATE_19: begin
                flag_14_to_15 <= 0;
                
                data_theta <= (65536 - p_cos_theta);
                data <= (65536 - p_cos);
                
                if(sqrt_valid_theta == 1 && sqrt_valid == 1) begin  
                    flag_19_to_20 <= 1;
                end
                else begin
                    flag_19_to_20 <= 0;
                end                
            end
            STATE_20: begin 
                flag_19_to_20 <= 0;
                
                if (sqrt_valid_theta == 1 && sqrt_valid == 1) begin
                    cordic_y_3 <= sqrt_theta;
                    cordic_x_3 <= a_cos_theta;
                    
                    cordic_y_4 <= sqrt;
                    cordic_x_4 <= quotient;
                    
                    if(cordic_valid_3 == 0 && cordic_valid_4 == 0) begin
                        flag_20_to_21 <= 1;
                    end
                end
                else begin
                    flag_20_to_21 <= 0;
                end
            end
            STATE_21: begin 
                flag_20_to_21 <= 0;
                
                if (cordic_valid_3 == 1 && cordic_valid_4 == 1) begin
                    if(i_y < (start_elevation - end_descent) && (p_x + p_y) > p_judge && a_cos_theta > 0) begin
                        raw_angle_3 <= 11796480 + cordic_output_3;
                    end
                    else begin
                        raw_angle_3 <= 11796480 - cordic_output_3;
                    end
                    
                    raw_angle_4 <= cordic_output_4;
                    
                    flag_21_to_22 <= 1;
                end
                else begin
                    flag_21_to_22 <= 0;
                end
            end
            STATE_22: begin 
                flag_21_to_22 <= 0;
                
                final_angle_3 <= (raw_angle_3 >> 8);
                final_angle_4 <= ((raw_angle_3 - raw_angle_4 + 2621440) >> 8);
            end
            default: ; 
        endcase
    end
end

endmodule
