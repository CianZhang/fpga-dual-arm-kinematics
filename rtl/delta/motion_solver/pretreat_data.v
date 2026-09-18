//三个主动臂角度同步解算，主动臂垂直向下为0°
//主动臂向外角度为正，范围0到150
//详细命名注释参考python程序

//输入的xyz放大16倍，ABC放大65536倍

//A1 = 2m(r - R) + 2mx = 1064960x - 682443407
//B1 = -2zm = -1064960z
//C1 = (x + r - R)^2 + y^2 + z^2 + m^2 - n^2 = (16x - 10253)^2 + (16y)^2 + (16z)^2 - 5436604416

//A2 = 2m(r - R) - mx + √3my = -532480x + 922282y - 682443407
//B2 = -2zm = -1064960z
//C2 = (r - R - 0.5x + 0.5√3y)^2 + (0.5√3x + 0.5y)^2 + z^2 + m^2 - n^2 = (16x)^2 + (16y)^2 + (16z)^2 + 164049x - 284141y - 5331479259

//A3 = 2m(r - R) - mx - √3my = -532480x - 922282y - 682443407
//B3 = -2zm = -1064960z
//C3 = (r - R - 0.5x - 0.5√3y)^2 + (0.5√3x - 0.5y)^2 + z^2 + m^2 - n^2 = (16x)^2 + (16y)^2 + (16z)^2 + 164049x + 284141y - 5331479259

module pretreat_data
(
    input wire                  sys_clk,                // 系统时钟输入，50MHz频率    
    input wire                  sys_rst_n,              // 复位信号输入，低电平复位

    input wire signed [15:0]    x_16,                   // 放大16倍的x坐标     
    input wire signed [15:0]    y_16,                   // 放大16倍的y坐标 
    input wire signed [15:0]    z_16,                   // 放大16倍的z坐标 
    
    output reg signed [63:0]    A_1,                    // 计算出的结果A1
    output reg signed [63:0]    B_1,                    // 计算出的结果B1
    output reg signed [63:0]    C_1,                    // 计算出的结果C1
    
    output reg signed [63:0]    A_2,                    // 计算出的结果A2
    output reg signed [63:0]    B_2,                    // 计算出的结果B2
    output reg signed [63:0]    C_2,                    // 计算出的结果C2
    
    output reg signed [63:0]    A_3,                    // 计算出的结果A3
    output reg signed [63:0]    B_3,                    // 计算出的结果B3
    output reg signed [63:0]    C_3,                    // 计算出的结果C3
    
    output reg                  work_done               // 电机1数据预处理完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

//定义数据并未直接使用，直接修改对程序无效，需要根据公式修改程序
localparam main_arm = 130;                              // 主动臂长度130
localparam follower_arm = 316;                          // 从动臂长度316
localparam static_latform_radius = 74.6920;             // 静平台中心到主动臂起始点长度74.6920
localparam moving_platform_radius = 34.6410;            // 动平台中心到主动臂起始点长度34.6410

reg  signed [63:0] p_B;                                 // -1064960 * z_16
reg  signed [63:0] p_A_1;                               // 1064960 * x_16
reg  signed [63:0] p_C_11;                              // (16x_16 - 10253)^2
reg  signed [63:0] p_C_12;                              // (16y_16)^2
reg  signed [63:0] p_C_13;                              // (16z_16)^2
reg  signed [63:0] p_A_21;                              // -532480 * x_16
reg  signed [63:0] p_A_22;                              // 922282 * y_16
reg  signed [63:0] p_C_21;                              // (16x_16)^2
reg  signed [63:0] p_C_22;                              // 164049 * x_16
reg  signed [63:0] p_C_23;                              // -284141 * y_16

reg                 ip_multiplier_work;
reg  signed  [31:0] a;                 
reg  signed  [31:0] b;                 
wire signed  [63:0] p;  

reg  signed  [15:0] current_x = 0;
reg  signed  [15:0] current_y = 0;                 
reg  signed  [15:0] current_z = 0;

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

reg          [4:0] state, next_state;
reg                flag_0_to_1;

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
        STATE_9:  next_state = STATE_10;
        STATE_10: next_state = STATE_11;
        STATE_11: next_state = STATE_12;
        STATE_12: next_state = STATE_13;
        STATE_13: next_state = STATE_14;
        STATE_14: next_state = STATE_15;
        STATE_15: next_state = STATE_16;
        STATE_16: next_state = STATE_17;
        STATE_17: next_state = STATE_18;
        STATE_18: next_state = STATE_19;
        STATE_19: next_state = STATE_20;
        STATE_20: next_state = STATE_21;
        STATE_21: next_state = STATE_0;
        default:  next_state = STATE_0;
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
        
        A_1 <= 0;
        B_1 <= 0;
        C_1 <= 0;
        A_2 <= 0;
        B_2 <= 0;
        C_2 <= 0;
        A_3 <= 0;
        B_3 <= 0;
        C_3 <= 0;
        
        work_done <= 1;
        
        p_B <= 0;     
        p_A_1 <= 0;
        p_C_11 <= 0;
        p_C_12 <= 0;
        p_C_13 <= 0;
        p_A_21 <= 0;
        p_A_22 <= 0;
        p_C_21 <= 0;
        p_C_22 <= 0;
        p_C_23 <= 0;
        
        flag_0_to_1 <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                if (x_16 != current_x || y_16 != current_y || z_16 != current_z) begin                  
                    work_done <= 0;
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            
            STATE_1: begin // p_B <= -(1064960 * z_16);
                ip_multiplier_work <= 1;
                a <= -1064960;
                b <= z_16;  
            end
            STATE_2: begin
                ip_multiplier_work <= 0;
                p_B <= p;
            end
            STATE_3: begin // p_A_1 <= 1064960 * x_16;
                ip_multiplier_work <= 1;
                a <= 1064960;
                b <= x_16;  
            end            
            STATE_4: begin
                ip_multiplier_work <= 0;
                p_A_1 <= p;
            end                
            STATE_5: begin // p_C_11 <= ((x_16 << 4) - 10253) * ((x_16 << 4) - 10253);
                ip_multiplier_work <= 1;
                a <= ((x_16 << 4) - 10253);
                b <= ((x_16 << 4) - 10253);  
            end            
            STATE_6: begin
                ip_multiplier_work <= 0;
                p_C_11 <= p;
            end                
            STATE_7: begin // p_C_12 <= (y_16 << 4) * (y_16 << 4);
                ip_multiplier_work <= 1;
                a <= (y_16 << 4);
                b <= (y_16 << 4);  
            end            
            STATE_8: begin
                ip_multiplier_work <= 0;
                p_C_12 <= p;
            end                    
            STATE_9: begin // p_C_13 <= (z_16 << 4) * (z_16 << 4); 
                ip_multiplier_work <= 1;
                a <= (z_16 << 4);
                b <= (z_16 << 4);  
            end            
            STATE_10: begin
                ip_multiplier_work <= 0;
                p_C_13 <= p;
            end
            STATE_11: begin // p_A_21 <= -(532480 * x_16);
                ip_multiplier_work <= 1;
                a <= -532480;
                b <= x_16;  
            end            
            STATE_12: begin
                ip_multiplier_work <= 0;
                p_A_21 <= p;
            end
            STATE_13: begin // p_A_22 <= 922282 * y_16;
                ip_multiplier_work <= 1;
                a <= 922282;
                b <= y_16;  
            end            
            STATE_14: begin
                ip_multiplier_work <= 0;
                p_A_22 <= p;
            end  
            STATE_15: begin // p_C_21 <= (x_16 << 4) * (x_16 << 4);
                ip_multiplier_work <= 1;
                a <= (x_16 << 4);
                b <= (x_16 << 4);  
            end            
            STATE_16: begin
                ip_multiplier_work <= 0;
                p_C_21 <= p;
            end 
            STATE_17: begin // p_C_22 <= 164049 * x_16; 
                ip_multiplier_work <= 1;
                a <= 164049;
                b <= x_16;  
            end            
            STATE_18: begin
                ip_multiplier_work <= 0;
                p_C_22 <= p;
            end  
            STATE_19: begin // p_C_23 <= -(284141 * y_16);
                ip_multiplier_work <= 1;
                a <= -284141;
                b <= y_16;  
            end            
            STATE_20: begin
                ip_multiplier_work <= 0;
                p_C_23 <= p;
            end            
                                       
            STATE_21: begin 
                flag_0_to_1 <= 0; 
                
                A_1 <= p_A_1 - 64'd682443407;
                B_1 <= p_B;
                C_1 <= p_C_11 + p_C_12 + p_C_13 - 64'd5436604416;

                A_2 <= p_A_21 + p_A_22 - 64'd682443407;
                B_2 <= p_B;
                C_2 <= p_C_21 + p_C_12 + p_C_13 + p_C_22 + p_C_23 - 64'd5331479259;

                A_3 <= p_A_21 - p_A_22 - 64'd682443407;
                B_3 <= p_B;
                C_3 <= p_C_21 + p_C_12 + p_C_13 + p_C_22 - p_C_23 - 64'd5331479259;
                
                current_x <= x_16;
                current_y <= y_16;
                current_z <= z_16;   
                work_done <= 1;
            end
            default: ; 
        endcase
    end
end   

endmodule 
