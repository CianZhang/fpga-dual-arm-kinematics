//用于计算 Asinr + Bcosr = C 的模块
//详细命名注释参考python程序

//Numerator = -A * B - C * (√(A^2 + B^2 - C^2))
//Denominator = A^2 - C^2
//tanr = Numerator / Denominator

//if(Quotient < 0):
//  result = math.atan(-Quotient)
//  angle = 180 - math.degrees(result)  
//if(Quotient > 0):   
//  result = math.atan(Quotient)
//  angle = math.degrees(result)
//return angle
    
module calculate_angle
(
    input wire                  sys_clk,                // 系统时钟输入，50MHz频率    
    input wire                  sys_rst_n,              // 复位信号输入，低电平复位
     
    input wire signed [63:0]    A,                      // 数据预处理的结果A，放大65536倍
    input wire signed [63:0]    B,                      // 数据预处理的结果B，放大65536倍
    input wire signed [63:0]    C,                      // 数据预处理的结果C，放大65536倍
    
    output reg        [31:0]    final_angle,            // 公式计算出的角度   
    
    output reg                  work_done               // 处理完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg  signed  [63:0] current_A = 0;
reg  signed  [63:0] current_B = 0;                 
reg  signed  [63:0] current_C = 0;

reg  signed  [63:0] Numerator;
reg  signed  [63:0] Denominator;

reg                 ip_multiplier_work;
reg  signed  [31:0] a;                 
reg  signed  [31:0] b;                 
wire signed  [63:0] p;                 

reg  signed  [63:0] p_N1; // -A * B
reg  signed  [63:0] p_N2; // -C * sqrt(A^2 + B^2 - C^2)
reg  signed  [63:0] p_N3; // A^2
reg  signed  [63:0] p_N4; // B^2
reg  signed  [63:0] p_N5; // C^2                

reg          [63:0] data; 
wire         [31:0] sqrt;
wire                sqrt_valid;

reg signed   [63:0] cordic_x;
reg signed   [63:0] cordic_y;
wire         [31:0] cordic_output;
wire                cordic_valid;

reg          [31:0] raw_angle;

reg          [1:0]  cnt;

parameter STATE_0  = 4'b0000;
parameter STATE_1  = 4'b0001;
parameter STATE_2  = 4'b0010;
parameter STATE_3  = 4'b0011;
parameter STATE_4  = 4'b0100;
parameter STATE_5  = 4'b0101;
parameter STATE_6  = 4'b0110;
parameter STATE_7  = 4'b0111;
parameter STATE_8  = 4'b1000;
parameter STATE_9  = 4'b1001;
parameter STATE_10 = 4'b1010;
parameter STATE_11 = 4'b1011;
parameter STATE_12 = 4'b1100;
parameter STATE_13 = 4'b1101;
parameter STATE_14 = 4'b1110;
parameter STATE_15 = 4'b1111;

reg          [3:0] state, next_state;
reg                flag_0_to_1;
reg                flag_9_to_10;
reg                flag_10_to_11;
reg                flag_12_to_13;
reg                flag_13_to_14;
reg                flag_14_to_15;
 
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

sqrt sqrt_inst(
    .sys_clk(sys_clk),       
    .sys_rst_n(sys_rst_n),     
    .data(data),        
    .sqrt(sqrt),     
    .valid(sqrt_valid)     
);

cordic_serial cordic_serial_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x(cordic_x),     
    .y(cordic_y),   
    .phase(cordic_output),
    .valid(cordic_valid)
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
        STATE_7:  next_state = STATE_8;
        STATE_8:  next_state = STATE_9;
        STATE_9:  next_state = flag_9_to_10 ? STATE_10 : STATE_9;
        STATE_10: next_state = flag_10_to_11 ? STATE_11 : STATE_10;
        STATE_11: next_state = STATE_12;
        STATE_12: next_state = flag_12_to_13 ? STATE_13 : STATE_12;
        STATE_13: next_state = flag_13_to_14 ? STATE_14 : STATE_13;
        STATE_14: next_state = flag_14_to_15 ? STATE_15 : STATE_14;
        STATE_15: next_state = STATE_0;
        default:  next_state = STATE_0;
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        current_A <= 0;
        current_B <= 0;                 
        current_C <= 0;
        raw_angle <= 0; 
        final_angle <= 0; 
        work_done <= 1;
        cnt <= 0;

        Numerator <= 0; 
        Denominator <= 0; 
        
        ip_multiplier_work <= 0;        
        a <= 0;
        b <= 0;        

        p_N1 <= 0;
        p_N2 <= 0;                 
        p_N3 <= 0;                 
        p_N4 <= 0;                 
        p_N5 <= 0;

        data <= 0;
        
        cordic_x <= 0;
        cordic_y <= 0;

        flag_0_to_1 <= 0;
        flag_9_to_10 <= 0;
        flag_10_to_11 <= 0;
        flag_12_to_13 <= 0;
        flag_13_to_14 <= 0;
        flag_14_to_15 <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                ip_multiplier_work <= 0;        
                
                // ABC放大65536倍，最小有符号35位宽能装下，在此处缩小16倍，有符号32位宽能装下
                if (A != current_A || B != current_B || C != current_C) begin
                    work_done <= 0;
                    
                    flag_0_to_1 <= 1;                    
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end                 
            STATE_1: begin // p_N1 = (A >> 4) * (B >> 4)
                ip_multiplier_work <= 1;
                a <= (A >> 4);
                b <= (B >> 4);  
            end 
            STATE_2: begin
                ip_multiplier_work <= 0;
                p_N1 <= p;
            end            
            STATE_3: begin // p_N3 = (A >> 4) * (A >> 4)
                ip_multiplier_work <= 1;
                a <= (A >> 4);
                b <= (A >> 4);  
            end 
            STATE_4: begin
                ip_multiplier_work <= 0;
                p_N3 <= p;
            end
            STATE_5: begin // p_N4 = (B >> 4) * (B >> 4)
                ip_multiplier_work <= 1;
                a <= (B >> 4);
                b <= (B >> 4);  
            end 
            STATE_6: begin
                ip_multiplier_work <= 0;
                p_N4 <= p;
            end
            STATE_7: begin // p_N5 = (C >> 4) * (C >> 4)
                ip_multiplier_work <= 1;
                a <= (C >> 4);
                b <= (C >> 4);  
            end 
            STATE_8: begin
                ip_multiplier_work <= 0;
                p_N5 <= p;
            end             
            STATE_9: begin // Start sqrt(A^2 + B^2 - C^2)
                flag_0_to_1 <= 0;
                
                data <= p_N3 + p_N4 - p_N5;
                
                if(sqrt_valid == 0) begin  
                    flag_9_to_10 <= 1;
                end
                else begin
                    flag_9_to_10 <= 0;
                end                
            end
            STATE_10: begin // p_N2 = -(C >> 4) * sqrt 
                flag_9_to_10 <= 0;
                
                if(sqrt_valid) begin
                    ip_multiplier_work <= 1;
                    a <= -(C >> 4);
                    b <= sqrt;
                    
                    flag_10_to_11 <= 1;
                end
                else begin
                    flag_10_to_11 <= 0;
                end
            end
            STATE_11: begin 
                ip_multiplier_work <= 0;
                p_N2 <= p;  
            end
            STATE_12: begin // Calculate Numerator and Denominator
                flag_10_to_11 <= 0;
                
                cnt <= cnt + 1;
                if(cnt) begin
                    Numerator <= p_N2 - p_N1;     
                    Denominator <= p_N3 - p_N5;
                    cnt <= 0;
                    flag_12_to_13 <= 1;                    
                end
                else begin
                   flag_12_to_13 <= 0; 
                end
            end
            STATE_13: begin
                flag_12_to_13 <= 0;
            
                if(Numerator > 0 && Denominator > 0) begin
                    cordic_y <= Numerator; 
                    cordic_x <= Denominator;
                end
                if(Numerator < 0 && Denominator < 0) begin
                    cordic_y <= -Numerator; 
                    cordic_x <= -Denominator;
                end                
                if(Numerator > 0 && Denominator < 0) begin
                    cordic_y <= Numerator; 
                    cordic_x <= -Denominator;
                end               
                if(Numerator < 0 && Denominator > 0) begin
                    cordic_y <= -Numerator; 
                    cordic_x <= Denominator;
                end 
                
                if(cordic_valid == 0) begin
                    flag_13_to_14 <= 1;
                end
                else begin
                    flag_13_to_14 <= 0;
                end
            end
            STATE_14: begin 
                flag_13_to_14 <= 0;
                
                if (cordic_valid) begin
                    if(Numerator > 0 && Denominator > 0) begin
                        raw_angle <= cordic_output;
                    end
                    if(Numerator < 0 && Denominator < 0) begin
                        raw_angle <= cordic_output;
                    end                
                    if(Numerator > 0 && Denominator < 0) begin
                        raw_angle <= 11796480 - cordic_output;
                    end               
                    if(Numerator < 0 && Denominator > 0) begin
                        raw_angle <= 11796480 - cordic_output;
                    end 
                    
                    flag_14_to_15 <= 1;
                end
                else begin
                    flag_14_to_15 <= 0;
                end
            end 
            STATE_15: begin 
                flag_14_to_15 <= 0;
                
                final_angle <= 33280 - (raw_angle >> 8);

                current_A <= A;
                current_B <= B;                 
                current_C <= C;                
                work_done <= 1;
            end
            default: ; 
        endcase
    end
end

endmodule
