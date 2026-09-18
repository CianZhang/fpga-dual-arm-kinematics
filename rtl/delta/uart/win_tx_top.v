module win_tx_top (
    input  wire            sys_clk,     // 50MHz系统时钟
    input  wire            sys_rst_n,   // 低电平有效复位
    input  wire            valid,       // 数据有效信号，高电平触发发送
    input  wire [7:0]      data1,       // 第一个 8 位数据
    input  wire [7:0]      data2,       // 第二个 8 位数据
    input  wire [7:0]      data3,       // 第三个 8 位数据
    input  wire [7:0]      data4,       // 第四个 8 位数据
    input  wire [7:0]      data5,       // 第五个 8 位数据
    input  wire [7:0]      data6,       // 第六个 8 位数据
    input  wire [7:0]      data7,       // 第七个 8 位数据
    input  wire [7:0]      data8,       // 第八个 8 位数据
    input  wire [7:0]      data9,       // 第九个 8 位数据
    input  wire [7:0]      data10,      // 第十个 8 位数据
    input  wire [7:0]      data11,      // 第十一个 8 位数据
    input  wire [7:0]      data12,      // 第十二个 8 位数据
    output wire            tx           // UART发送输出
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam UART_BPS    = 9600;          // 波特率
localparam CLK_FREQ    = 50;            // 时钟频率，单位MHz

wire                tx_done;            // UART发送完成

reg [7:0]   data1_reg;
reg [7:0]   data2_reg; 
reg [7:0]   data3_reg; 
reg [7:0]   data4_reg;
reg [7:0]   data5_reg;
reg [7:0]   data6_reg;
reg [7:0]   data7_reg;
reg [7:0]   data8_reg;
reg [7:0]   data9_reg;
reg [7:0]   data10_reg;
reg [7:0]   data11_reg;
reg [7:0]   data12_reg;

reg [7:0]   pi_data;          // 发送数据
reg         pi_flag;          // 数据有效标志

parameter STATE_0  = 4'b0000; // 等待valid信号
parameter STATE_1  = 4'b0001; // 发送包头0xBB
parameter STATE_2  = 4'b0010; // 发送data1
parameter STATE_3  = 4'b0011; // 发送data2
parameter STATE_4  = 4'b0100; // 发送data3
parameter STATE_5  = 4'b0101; // 发送data4
parameter STATE_6  = 4'b0110; // 发送data5
parameter STATE_7  = 4'b0111; // 发送data6
parameter STATE_8  = 4'b1000; // 发送data7
parameter STATE_9  = 4'b1001; // 发送data8
parameter STATE_10 = 4'b1010; // 发送data9
parameter STATE_11 = 4'b1011; // 发送data10
parameter STATE_12 = 4'b1100; // 发送data11
parameter STATE_13 = 4'b1101; // 发送data12
parameter STATE_14 = 4'b1110; // 发送包尾0x55

reg [3:0]   state, next_state; 
reg         flag_0_to_1;
reg         flag_1_to_2;
reg         flag_2_to_3;
reg         flag_3_to_4;
reg         flag_4_to_5;
reg         flag_5_to_6;
reg         flag_6_to_7;
reg         flag_7_to_8;
reg         flag_8_to_9;
reg         flag_9_to_10;
reg         flag_10_to_11;
reg         flag_11_to_12;
reg         flag_12_to_13;
reg         flag_13_to_14;
reg         flag_14_to_0;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

uart_tx #(
    .CLK_FRE(CLK_FREQ),
    .BAUD_RATE(UART_BPS)
) u_uart_tx (
    .clk(sys_clk),
    .rst_n(sys_rst_n),
    .tx_data(pi_data),
    .tx_data_valid(pi_flag),
    .tx_data_ready(tx_done),
    .tx_pin(tx)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

// 状态寄存器
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state <= STATE_0; // 复位时进入初始状态
    end else begin
        state <= next_state; // 状态更新
    end
end

// 下一状态逻辑
always @(*) begin
    case (state)
        STATE_0:  next_state = flag_0_to_1  ? STATE_1  : STATE_0;
        STATE_1:  next_state = flag_1_to_2  ? STATE_2  : STATE_1;
        STATE_2:  next_state = flag_2_to_3  ? STATE_3  : STATE_2;
        STATE_3:  next_state = flag_3_to_4  ? STATE_4  : STATE_3;
        STATE_4:  next_state = flag_4_to_5  ? STATE_5  : STATE_4;
        STATE_5:  next_state = flag_5_to_6  ? STATE_6  : STATE_5;
        STATE_6:  next_state = flag_6_to_7  ? STATE_7  : STATE_6;
        STATE_7:  next_state = flag_7_to_8  ? STATE_8  : STATE_7;
        STATE_8:  next_state = flag_8_to_9  ? STATE_9  : STATE_8;
        STATE_9:  next_state = flag_9_to_10 ? STATE_10 : STATE_9;
        STATE_10: next_state = flag_10_to_11 ? STATE_11 : STATE_10;
        STATE_11: next_state = flag_11_to_12 ? STATE_12 : STATE_11;
        STATE_12: next_state = flag_12_to_13 ? STATE_13 : STATE_12;
        STATE_13: next_state = flag_13_to_14 ? STATE_14 : STATE_13;
        STATE_14: next_state = flag_14_to_0  ? STATE_0  : STATE_14;
        default:  next_state = STATE_0;
    endcase
end

// 数据和标志控制
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pi_data <= 8'h00;
        pi_flag <= 1'b0;
        
        data1_reg  <= 8'h00;
        data2_reg  <= 8'h00;
        data3_reg  <= 8'h00;
        data4_reg  <= 8'h00;
        data5_reg  <= 8'h00;
        data6_reg  <= 8'h00;
        data7_reg  <= 8'h00;
        data8_reg  <= 8'h00;
        data9_reg  <= 8'h00;
        data10_reg <= 8'h00;
        data11_reg <= 8'h00;
        data12_reg <= 8'h00;
        
        flag_0_to_1   <= 1'b0;
        flag_1_to_2   <= 1'b0;
        flag_2_to_3   <= 1'b0;
        flag_3_to_4   <= 1'b0;
        flag_4_to_5   <= 1'b0;
        flag_5_to_6   <= 1'b0;
        flag_6_to_7   <= 1'b0;
        flag_7_to_8   <= 1'b0;
        flag_8_to_9   <= 1'b0;
        flag_9_to_10  <= 1'b0;
        flag_10_to_11 <= 1'b0;
        flag_11_to_12 <= 1'b0;
        flag_12_to_13 <= 1'b0;
        flag_13_to_14 <= 1'b0;
        flag_14_to_0  <= 1'b0;
    end 
    else begin
        case (state)
            STATE_0: begin
                flag_14_to_0 <= 1'b0;
                pi_flag <= 1'b0;
                
                if (valid) begin
                    data1_reg <= data1;
                    data2_reg <= data2;
                    data3_reg <= data3;
                    data4_reg <= data4;
                    data5_reg <= data5;
                    data6_reg <= data6;
                    data7_reg <= data7;
                    data8_reg <= data7;
                    data9_reg <= data9;
                    data10_reg <= data10;
                    data11_reg <= data11;
                    data12_reg <= data12;
                    
                    flag_0_to_1 <= 1'b1;
                end else begin
                    flag_0_to_1 <= 1'b0;
                end
            end
            STATE_1: begin
                flag_0_to_1 <= 1'b0;
                
                pi_data <= 8'hBB; // 发送包头
                pi_flag <= 1'b1;
                
                flag_1_to_2 <= 1'b1;
            end
            STATE_2: begin
                flag_1_to_2 <= 1'b0;
                
                pi_flag <= 1'b0;
                
                if (tx_done) begin
                    pi_data <= data1_reg; // data1
                    pi_flag <= 1'b1;
                    
                    flag_2_to_3 <= 1'b1;
                end 
                else begin
                    flag_2_to_3 <= 1'b0;
                end
            end
            STATE_3: begin
                flag_2_to_3 <= 1'b0;
                
                pi_flag <= 1'b0;
                
                if (tx_done) begin
                    pi_data <= data2_reg; // data2
                    pi_flag <= 1'b1;
                    
                    flag_3_to_4 <= 1'b1;
                end 
                else begin
                    flag_3_to_4 <= 1'b0;
                end
            end
            STATE_4: begin
                flag_3_to_4 <= 1'b0;
                
                pi_flag <= 1'b0;
                
                if (tx_done) begin
                    pi_data <= data3_reg; // data3
                    pi_flag <= 1'b1;
                    
                    flag_4_to_5 <= 1'b1;
                end 
                else begin
                    flag_4_to_5 <= 1'b0;
                end
            end
            STATE_5: begin
                flag_4_to_5 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data4_reg; // data4
                    pi_flag <= 1'b1;
                    flag_5_to_6 <= 1'b1;
                end else begin
                    flag_5_to_6 <= 1'b0;
                end
            end
            STATE_6: begin
                flag_5_to_6 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data5_reg; // data5
                    pi_flag <= 1'b1;
                    flag_6_to_7 <= 1'b1;
                end else begin
                    flag_6_to_7 <= 1'b0;
                end
            end
            STATE_7: begin
                flag_6_to_7 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data6_reg; // data6
                    pi_flag <= 1'b1;
                    flag_7_to_8 <= 1'b1;
                end else begin
                    flag_7_to_8 <= 1'b0;
                end
            end
            STATE_8: begin
                flag_7_to_8 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data7_reg; // data7
                    pi_flag <= 1'b1;
                    flag_8_to_9 <= 1'b1;
                end else begin
                    flag_8_to_9 <= 1'b0;
                end
            end
            STATE_9: begin
                flag_8_to_9 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data8_reg; // data8
                    pi_flag <= 1'b1;
                    flag_9_to_10 <= 1'b1;
                end else begin
                    flag_9_to_10 <= 1'b0;
                end
            end
            STATE_10: begin
                flag_9_to_10 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data9_reg; // data9
                    pi_flag <= 1'b1;
                    flag_10_to_11 <= 1'b1;
                end else begin
                    flag_10_to_11 <= 1'b0;
                end
            end
            STATE_11: begin
                flag_10_to_11 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data10_reg; // data10
                    pi_flag <= 1'b1;
                    flag_11_to_12 <= 1'b1;
                end else begin
                    flag_11_to_12 <= 1'b0;
                end
            end
            STATE_12: begin
                flag_11_to_12 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data11_reg; // data11
                    pi_flag <= 1'b1;
                    flag_12_to_13 <= 1'b1;
                end else begin
                    flag_12_to_13 <= 1'b0;
                end
            end
            STATE_13: begin
                flag_12_to_13 <= 1'b0;
                pi_flag <= 1'b0;
                if (tx_done) begin
                    pi_data <= data12_reg; // data12
                    pi_flag <= 1'b1;
                    flag_13_to_14 <= 1'b1;
                end else begin
                    flag_13_to_14 <= 1'b0;
                end
            end
            STATE_14: begin
                flag_13_to_14 <= 1'b0;
                
                pi_flag <= 1'b0;
                
                if (tx_done) begin
                    pi_data <= 8'h55; // 发送包尾
                    pi_flag <= 1'b1;
                    
                    flag_14_to_0 <= 1'b0;
                end 
                else begin
                    flag_14_to_0 <= 1'b0;
                end
            end
            default: begin
                pi_data <= 8'h00;
                pi_flag <= 1'b0;
                flag_0_to_1 <= 1'b0;
                flag_1_to_2 <= 1'b0;
                flag_2_to_3 <= 1'b0;
                flag_3_to_4 <= 1'b0;
                flag_4_to_5 <= 1'b0;
                flag_5_to_6 <= 1'b0;
                flag_6_to_7 <= 1'b0;
                flag_7_to_8 <= 1'b0;
                flag_8_to_9 <= 1'b0;
                flag_9_to_10 <= 1'b0;
                flag_10_to_11 <= 1'b0;
                flag_11_to_12 <= 1'b0;
                flag_12_to_13 <= 1'b0;
                flag_13_to_14 <= 1'b0;
                flag_14_to_0 <= 1'b0;
            end
        endcase
    end
end            

endmodule