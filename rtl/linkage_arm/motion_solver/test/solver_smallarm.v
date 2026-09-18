//小臂角度的计算
//输入变换后的坐标X，Y计算出角度，其中X = 真正的 根号下(X^2 + Y^2)，Y = 真正的 Z
//大臂与小臂的夹角，逆时针为正

module solver_smallarm
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

reg signed [31:0] current_y = 0;                 
reg signed [31:0] current_x = 0;                  

reg signed [31:0] cordic_x;
reg signed [31:0] cordic_y;
wire       [31:0] cordic_output;
wire              cordic_valid;

reg signed [31:0] raw_angle;
reg signed [31:0] final_angle;

wire       [31:0] p_x;           
reg        [15:0] a_x;           
reg        [15:0] b_x;
           
wire       [31:0] p_y;           
reg        [15:0] a_y;           
reg        [15:0] b_y;   
        
wire       [31:0] p_cos;           
reg        [15:0] a_cos;           
reg        [15:0] b_cos;  

reg        [31:0] data;
wire       [15:0] sqrt;
wire              sqrt_valid;

reg        [31:0] denominator;                          //分母
reg        [31:0] numerator;                            //分子     
reg               start;                                //
wire              done;                                 //完成标志
wire       [31:0] quotient;                             //商
wire       [31:0] remainder;                            //余数

parameter STATE_0 = 3'b000;                             // 计算新的平方值
parameter STATE_1 = 3'b001;                             // 对新的平方值赋值
parameter STATE_2 = 3'b010;                             // 
parameter STATE_3 = 3'b011;                             //
parameter STATE_4 = 3'b100;                             //
parameter STATE_5 = 3'b101;

reg        [2:0]  state, next_state;                    // 当前状态和下一个状态寄存器
reg               flag_3_to_4;
reg               flag_4_to_5;
reg               flag_5_to_0;

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

ip_multiplier ip_multiplier_cos_inst (
    .p(p_cos),
    .a(a_cos),
    .b(b_cos)    
);

ip_divider ip_divider_inst (
    .clk(sys_clk),
    .denominator(denominator),
    .numerator(numerator),     
    .rst(~sys_rst_n),   
    .start(start),
    .done(done),
    .quotient(quotient),
    .remainder(remainder)
);

sqrt sqrt_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data),        
    .sqrt(sqrt),     
    .valid(sqrt_valid)     
);

cordic_serial cordic_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x),     
    .y(cordic_y),   
    .phase(cordic_output),
    .valid(cordic_valid)
);

stepper_motor_ctrl_4 stepper_motor_ctrl_4_inst (
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
            if(start == 1)
                next_state = STATE_1; // 如果start == 1，转到STATE_1
            else 
                next_state = STATE_0; // 否则保持在STATE_0
        end
        STATE_1: begin
            if(done == 0) 
                next_state = STATE_2; // 如果done == 0，转到STATE_2
            else 
                next_state = STATE_1; // 否则保持在STATE_1
        end
        STATE_2: begin
            if(done == 1) 
                next_state = STATE_3; // 如果done == 1，转到STATE_3
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
            if(flag_5_to_0 == 1) 
                next_state = STATE_0; // 如果flag_5_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_5; // 否则保持在STATE_5
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
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_0 <= 0;
                 
        a_x <= 0;           
        b_x <= 0;                    
        a_y <= 0;           
        b_y <= 0;                      
        a_cos <= 0;           
        b_cos <= 0;
        
        denominator <= 0;
        numerator <= 0; 
        start <= 0;        
    end
    else begin
        case(state)
            STATE_0: begin 
                flag_5_to_0 <= 0;
                if(i_x != current_x || i_y != current_y) begin
                    current_x <= i_x;
                    current_y <= i_y;
                    //分类讨论不同的三角形
                    if (i_y > (start_elevation - end_descent)) begin
                        // 赋值给ip_multiplier模块进行计算
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= (i_y + end_descent - start_elevation);         
                        b_y <= (i_y + end_descent - start_elevation);
                    end
                    else if (i_y < (start_elevation - end_descent)) begin
                        // 赋值给ip_multiplier模块进行计算
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= (start_elevation - i_y - end_descent);         
                        b_y <= (start_elevation - i_y - end_descent);
                    end
                    else begin
                        // 赋值给ip_multiplier模块进行计算
                        a_x <= (i_x - end_extension);         
                        b_x <= (i_x - end_extension);
                        
                        a_y <= 0;         
                        b_y <= 0;
                    end
                    start <= 1;// 准备输入除法器的值
                end
            end
            STATE_1: begin               
                numerator <= ((big_arm_length_2 + small_arm_length_2 - p_x - p_y) << 8);
                denominator <= small_and_big_2;
            end
            STATE_2: begin 
                start <= 0;// 除法器开始计算
                if (done) begin
                    a_cos <= quotient;
                    b_cos <= quotient;
                end
            end
            STATE_3: begin
                data <= (65536 - p_cos);
                if(sqrt_valid) begin  
                    flag_3_to_4 <= 1;
                end
                else begin
                    flag_3_to_4 <= 0;
                end
            end
            STATE_4: begin 
                flag_3_to_4 <= 0;
                if (sqrt_valid) begin
                    cordic_y <= sqrt;
                    cordic_x <= quotient;
                    if(cordic_valid == 0) begin
                        flag_4_to_5 <= 1;
                    end
                end
                else begin
                    flag_4_to_5 <= 0;
                end
            end
            STATE_5: begin 
                flag_4_to_5 <= 0;
                if (reached_target && cordic_valid) begin
                    raw_angle <= (cordic_output >> 8);
                    if(quotient < 0) begin
                        final_angle <= ((46080 - cordic_output) >> 8);
                    end
                    else begin
                        final_angle <= (cordic_output >> 8);
                    end
                    flag_5_to_0 <= 1;
                end
                else begin
                    flag_5_to_0 <= 0;
                end
            end
            default: ; 
        endcase
    end
end

endmodule
