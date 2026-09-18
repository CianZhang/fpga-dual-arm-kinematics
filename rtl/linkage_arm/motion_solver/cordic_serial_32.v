//cordic算法，向量模式，串行运行，大约需要14个时钟周期
//输出角度范围：（0 ~ 360）* 65536
//当valid拉高时，表示输出信号有效

module cordic_serial_32(
    input                  sys_clk,
    input                  sys_rst_n,
    
    input signed [31:0]    x,       // cos值
    input signed [31:0]    y,       // sin值

    output reg   [31:0]    phase,   // 输出角度值，最大360*65536
    output reg             valid    // 输出信号有效性
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam IDLE = 2'd0;
localparam WORK = 2'd1;
localparam DONE = 2'd2;

reg     [1:0]state ;
reg     [1:0]next_state;

reg     [6:0]counter;

wire    [31:0] rot[15:0];

// 旋转角度查找表
assign  rot[0]  = 32'd2949120 ;     //45度*2^16
assign  rot[1]  = 32'd1740992 ;     //26.5651度*2^16
assign  rot[2]  = 32'd919872  ;     //14.0362度*2^16
assign  rot[3]  = 32'd466944  ;     //7.1250度*2^16
assign  rot[4]  = 32'd234368  ;     //3.5763度*2^16
assign  rot[5]  = 32'd117312  ;     //1.7899度*2^16
assign  rot[6]  = 32'd58688   ;     //0.8952度*2^16
assign  rot[7]  = 32'd29312   ;     //0.4476度*2^16
assign  rot[8]  = 32'd14656   ;     //0.2238度*2^16
assign  rot[9]  = 32'd7360    ;     //0.1119度*2^16
assign  rot[10] = 32'd3648    ;     //0.0560度*2^16
assign  rot[11] = 32'd1856    ;     //0.0280度*2^16
assign  rot[12] = 32'd896     ;     //0.0140度*2^16
assign  rot[13] = 32'd448     ;     //0.0070度*2^16
assign  rot[14] = 32'd256     ;     //0.0035度*2^16
assign  rot[15] = 32'd128     ;     //0.0018度*2^16

// 定义上一个x和y的寄存器
reg signed [31:0] prev_x;
reg signed [31:0] prev_y;

reg signed [31:0] preprocessed_x;
reg signed [31:0] preprocessed_y;
reg               second_quadrant;
reg               third_quadrant;
reg               fourth_quadrant;

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

// 输入预处理
always @(*) begin
    second_quadrant = 1'b0;
    third_quadrant = 1'b0;
    fourth_quadrant = 1'b0;
    preprocessed_x = x;
    preprocessed_y = y;
    
    if (x < 0 && y > 0) begin
        // 第二象限
        preprocessed_x = -x;
        second_quadrant = 1'b1;
    end
    else if (x < 0 && y < 0) begin
        // 第三象限
        preprocessed_x = -x;
        preprocessed_y = -y;
        third_quadrant = 1'b1;
    end
    else if (x > 0 && y < 0) begin
        // 第四象限
        fourth_quadrant = 1'b1;
    end
end

// 状态机实现
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= IDLE;  // 复位时状态初始化
    end
    else begin
        state <= next_state;
    end
end

always @(*) begin
    case(state)
    IDLE: 
        if (x != prev_x || y != prev_y)
            next_state <= WORK;
        else
            next_state <= IDLE; 
    WORK: 
        next_state <= (counter == 14) ? DONE : WORK;
    DONE: 
        next_state <= IDLE;
    default: 
        next_state <= IDLE;
    endcase
end

reg signed [31:0] x_shift;
reg signed [31:0] y_shift;
reg signed [31:0] z_rot;

wire     D_sign;
assign   D_sign = ~y_shift[31];

// 工作状态逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        x_shift <= 0;
        y_shift <= 0;
        z_rot   <= 0;
        prev_x  <= 0;
        prev_y  <= 0;
        valid   <= 1'b0;
    end
    else begin
        case(state)
            IDLE: begin
                valid   <= 1'b1;
                x_shift <= preprocessed_x; 
                y_shift <= preprocessed_y;
                z_rot   <= 0;
                prev_x  <= x;
                prev_y  <= y;
            end
            WORK: begin
                valid <= 1'b0;
                if (D_sign) begin
                    x_shift <= x_shift + (y_shift >>> counter);
                    y_shift <= y_shift - (x_shift >>> counter);
                    z_rot   <= z_rot  + rot[counter];
                end
                else begin
                    x_shift <= x_shift - (y_shift >>> counter);
                    y_shift <= y_shift + (x_shift >>> counter);
                    z_rot   <= z_rot  - rot[counter];
                end
            end
            DONE: begin
                valid <= 1'b1;
                // 处理最终结果
                if (second_quadrant) begin
                    phase <= 32'd11796480 - z_rot;  // 180° * 2^16 - 计算结果
                end
                else if (third_quadrant) begin
                    phase <= z_rot + 32'd11796480;  // 计算结果 + 180° * 2^16
                end
                else if (fourth_quadrant) begin
                    phase <= 32'd23592960 + z_rot;  // 360° * 2^16 + 计算结果
                end
                else begin
                    phase <= z_rot;  // 第一的结果直接输出
                end
            end
            default: ;
        endcase
    end
end

// 计数器逻辑
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        counter <= 0;
    end
    else if (state == IDLE && next_state == WORK) begin
        counter <= 0;
    end
    else if (state == WORK) begin
        if (counter < 4'd14)
            counter <= counter + 1;
    end
    else begin
        counter <= 0;
    end
end

endmodule
