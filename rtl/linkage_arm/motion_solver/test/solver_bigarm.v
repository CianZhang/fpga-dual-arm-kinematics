//大臂角度的计算
//输入变换后的坐标X，Y计算出角度，其中X = 真正的 根号下(X^2 + Y^2)，Y = 真正的 Z
//大臂与X轴负半轴的夹角，顺时针为正

module solver_bigarm
(
    input wire                  sys_clk,      
    input wire                  sys_rst_n, 

    input wire signed [31:0]    i_x,                    // 计算之后的X     
    input wire signed [31:0]    i_y,                    // 计算之后的Y
  
    output wire                 step_pulse,             // 步进电机的脉冲信号输出
    output wire                 direction,              // 步进电机的方向信号输出
    output wire                 reached_target          // 目标位置标志位
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam start_elevation = 95;                         // 起始升高量
localparam big_arm_length = 150;                         // 大臂长
localparam small_arm_length = 150;                       // 小臂长
localparam end_extension = 105;                          // 末端前伸量
localparam end_descent = 60;                             // 末端下降量

localparam big_arm_length_2 = 22500;                     // 大臂长的平方
localparam small_arm_length_2 = 22500;                   // 小臂长的平方
localparam small_and_big_2 = 45000;                      // 2 * 大臂长 * 小臂长

wire       [31:0] p_x;                                  //边长平方       
reg        [15:0] a_x;           
reg        [15:0] b_x;
           
wire       [31:0] p_y;                                  //边长平方
reg        [15:0] a_y;           
reg        [15:0] b_y;

reg        [31:0] data_length;                          //对边长开方
wire       [15:0] sqrt_length;
wire              sqrt_valid_length;

reg        [31:0] data_sina;                            //对sina的分子开方
wire       [15:0] sqrt_sina;
wire              sqrt_valid_sina;

reg        [31:0] denominator_cosa_cosb;                //分母
reg        [31:0] numerator_cosa_cosb;                  //分子     
reg               start_cosa_cosb;                      //启动信号
wire              done_cosa_cosb;                       //完成标志
wire       [31:0] quotient_cosa_cosb;                   //商
wire       [31:0] remainder_cosa_cosb;                  //余数   
 
reg        [31:0] denominator_sina_sinb;                //分母
reg        [31:0] numerator_sina_sinb;                  //分子     
reg               start_sina_sinb;                      //启动信号
wire              done_sina_sinb;                       //完成标志
wire       [31:0] quotient_sina_sinb;                   //商
wire       [31:0] remainder_sina_sinb;                  //余数
 
wire signed[31:0] p_sina_sinb_numerator;                //sina_sinb的分子相乘      
reg signed [15:0] a_sina_sinb_numerator;                         
reg signed [15:0] b_sina_sinb_numerator;  

wire signed[31:0] p_sina_sinb_denominator;              //sina_sinb的分母相乘
reg signed [15:0] a_sina_sinb_denominator;           
reg signed [15:0] b_sina_sinb_denominator;

wire signed[31:0] p_cos_theta;                          //对大臂角平方
reg signed [15:0] a_cos_theta;           
reg signed [15:0] b_cos_theta;

reg        [31:0] data_theta;                           //对1 - 大臂角开方
wire       [15:0] sqrt_theta;
wire              sqrt_valid_theta;

reg signed [31:0] current_y = 0;                 
reg signed [31:0] current_x = 0;                  

reg signed [31:0] cordic_x;
reg signed [31:0] cordic_y;
wire       [31:0] cordic_output;
wire              cordic_valid;

reg signed [31:0] raw_angle;
reg signed [31:0] final_angle;

parameter STATE_0 = 3'b000;                           
parameter STATE_1 = 3'b001;                
parameter STATE_2 = 3'b010;                      
parameter STATE_3 = 3'b011;            
parameter STATE_4 = 3'b100;                           
parameter STATE_5 = 3'b101;
parameter STATE_6 = 3'b110;
parameter STATE_7 = 3'b111;

reg        [2:0]  state, next_state;                    // 当前状态和下一个状态寄存器
reg               flag_0_to_1;
reg               flag_1_to_2;
reg               flag_4_to_5;
reg               flag_5_to_6;
reg               flag_6_to_7;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

ip_multiplier ip_multiplier_x_inst (
    .p(p_x),
    .a(a_x),
    .b(b_x)    
);

ip_multiplier ip_multiplier_y_inst (
    .p(p_y),
    .a(a_y),
    .b(b_y)    
);

sqrt sqrt_length_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_length),        
    .sqrt(sqrt_length),     
    .valid(sqrt_valid_length)     
);

sqrt sqrt_sina_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_sina),        
    .sqrt(sqrt_sina),     
    .valid(sqrt_valid_sina)     
);

ip_divider ip_divider_cosa_cosb_inst (
    .clk(sys_clk),
    .denominator(denominator_cosa_cosb),
    .numerator(numerator_cosa_cosb),     
    .rst(~sys_rst_n),   
    .start(start_cosa_cosb),
    .done(done_cosa_cosb),
    .quotient(quotient_cosa_cosb),
    .remainder(remainder_cosa_cosb)
);

ip_divider ip_divider_sina_sinb_inst (
    .clk(sys_clk),
    .denominator(denominator_sina_sinb),
    .numerator(numerator_sina_sinb),     
    .rst(~sys_rst_n),   
    .start(start_sina_sinb),
    .done(done_sina_sinb),
    .quotient(quotient_sina_sinb),
    .remainder(remainder_sina_sinb)
);

ip_multiplier ip_multiplier_sina_sinb_numerator_inst (
    .p(p_sina_sinb_numerator),
    .a(a_sina_sinb_numerator),
    .b(b_sina_sinb_numerator)    
);

ip_multiplier ip_multiplier_sina_sinb_denominator_inst (
    .p(p_sina_sinb_denominator),
    .a(a_sina_sinb_denominator),
    .b(b_sina_sinb_denominator)    
);

ip_multiplier ip_multiplier_cos_theta_inst (
    .p(p_cos_theta),
    .a(a_cos_theta),
    .b(b_cos_theta)    
);

sqrt sqrt_theta_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data_theta),        
    .sqrt(sqrt_theta),     
    .valid(sqrt_valid_theta)     
);

cordic_serial cordic_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x),     
    .y(cordic_y),   
    .phase(cordic_output),
    .valid(cordic_valid)
);

stepper_motor_ctrl_3 stepper_motor_ctrl_3_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .target_angles(final_angle),
    .step_pulse(step_pulse),
    .direction(direction),
    .reached_target(reached_target)
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
            if(done_sina_sinb == 0) 
                next_state = STATE_3; // 如果done_sina_sinb == 0，转到STATE_3
            else 
                next_state = STATE_2; // 否则保持在STATE_2
        end
        STATE_3: begin
            if(done_sina_sinb == 1) 
                next_state = STATE_4; // 如果done_cosa_cosb == 1 && done_sina_sinb = 1，转到STATE_4
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
            next_state = STATE_0;
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cordic_x <= 0;
        cordic_y <= 0;
        raw_angle <= 0;
        final_angle <= 0;
        current_x <= 0;
        current_y <= 0;
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_6 <= 0;
        flag_6_to_7 <= 0;
                 
        a_x <= 0;           
        b_x <= 0;                    
        a_y <= 0;           
        b_y <= 0;
        a_cos_theta <= 0;           
        b_cos_theta <= 0;
        a_sina_sinb_numerator <= 0;           
        b_sina_sinb_numerator <= 0;        
        a_sina_sinb_denominator <= 0;           
        b_sina_sinb_denominator <= 0;
        
        denominator_cosa_cosb <= 0;
        numerator_cosa_cosb <= 0; 
        start_cosa_cosb <= 0;  
        denominator_sina_sinb <= 0;
        numerator_sina_sinb <= 0; 
        start_sina_sinb <= 0;         
    end
    else begin
        case(state)
            STATE_0: begin 
                start_cosa_cosb <= 1;
                if(i_x != current_x || i_y != current_y) begin
                    current_x <= i_x;
                    current_y <= i_y;
                    if (i_y > (start_elevation - end_descent)) begin
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= (i_y + end_descent - start_elevation);         
                        b_y <= (i_y + end_descent - start_elevation);
                    end
                    else if (i_y < (start_elevation - end_descent)) begin
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= (start_elevation - i_y - end_descent);         
                        b_y <= (start_elevation - i_y - end_descent);
                    end
                    else begin
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= 0;         
                        b_y <= 0;
                    end
                    numerator_cosa_cosb <= ((i_x - end_extension) << 8);
                    denominator_cosa_cosb <= 300;   //2 * big_arm_length
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin
                flag_0_to_1 <= 0;
                start_cosa_cosb <= 0; 
                data_length <= p_x + p_y;
                data_sina <= 90000 - p_x - p_y;     //4 * big_arm_length_2
                if(sqrt_valid_length == 1 && sqrt_valid_sina == 1) begin  
                    if (i_y > (start_elevation - end_descent)) begin
                        a_sina_sinb_numerator <= sqrt_sina;         
                        b_sina_sinb_numerator <= (i_y + end_descent - start_elevation);
                        
                        a_sina_sinb_denominator <= 300;   //2 * big_arm_length      
                        b_sina_sinb_denominator <= sqrt_length; 
                    end
                    else if (i_y < (start_elevation - end_descent)) begin
                        a_sina_sinb_numerator <= sqrt_sina;         
                        b_sina_sinb_numerator <= (start_elevation - i_y - end_descent);
                        
                        a_sina_sinb_denominator <= 300;   //2 * big_arm_length          
                        b_sina_sinb_denominator <= sqrt_length; 
                    end
                    else begin
                        a_sina_sinb_numerator <= 0;         
                        b_sina_sinb_numerator <= 0;
                        
                        a_sina_sinb_denominator <= 300;   //2 * big_arm_length          
                        b_sina_sinb_denominator <= sqrt_length; 
                    end                     
                    start_sina_sinb <= 1;
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end
            
            STATE_2: begin
                flag_1_to_2 <= 0;
                numerator_sina_sinb <= (p_sina_sinb_numerator << 8);  
                denominator_sina_sinb <= p_sina_sinb_denominator; 
            end
            STATE_3: begin 
                start_sina_sinb <= 0;
                if (done_sina_sinb == 1) begin
                    if (i_y > (start_elevation - end_descent)) begin
                        a_cos_theta <= (quotient_cosa_cosb - quotient_sina_sinb);
                        b_cos_theta <= (quotient_cosa_cosb - quotient_sina_sinb);
                    end
                    else if (i_y < (start_elevation - end_descent)) begin
                        a_cos_theta <= (quotient_cosa_cosb + quotient_sina_sinb);
                        b_cos_theta <= (quotient_cosa_cosb + quotient_sina_sinb);
                    end
                    else begin
                        a_cos_theta <= quotient_cosa_cosb;
                        b_cos_theta <= quotient_cosa_cosb;
                    end 
                end
            end
            STATE_4: begin
                data_theta <= (65536 - p_cos_theta);
                if(sqrt_valid_theta) begin  
                    flag_4_to_5 <= 1;
                end
                else begin
                    flag_4_to_5 <= 0;
                end                
            end
            STATE_5: begin 
                flag_4_to_5 <= 0;
                if (sqrt_valid_theta) begin
                    cordic_y <= sqrt_theta;
                    cordic_x <= a_cos_theta;
                    if(cordic_valid == 0) begin
                        flag_5_to_6 <= 1;
                    end
                end
                else begin
                    flag_5_to_6 <= 0;
                end
            end
            STATE_6: begin 
                flag_5_to_6 <= 0;
                if (reached_target && cordic_valid) begin
                    raw_angle <= cordic_output;
                    if(a_cos_theta < 0) begin
                        raw_angle <= 46080 - cordic_output;
                    end
                    else begin
                        raw_angle <= cordic_output;
                    end
                    flag_6_to_7 <= 1;
                end
                else begin
                    flag_6_to_7 <= 0;
                end
            end
            STATE_7: begin 
                flag_6_to_7 <= 0;
                final_angle <= (46080 - raw_angle >> 8);
            end
            default: ; 
        endcase
    end
end

endmodule
