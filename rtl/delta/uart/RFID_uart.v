module RFID_uart
#(
    parameter UART_BPS     = 'd115200,       // 串口波特率
    parameter CLK_FREQ     = 'd50_000_000    // 时钟频率
)
(
    input  wire              sys_clk,        // 系统时钟50MHz
    input  wire              sys_rst_n,      // 全局复位，低有效
    input  wire              rx,             // 串口接收数据
    
    output reg   [2:0]       match_id        // 输出物品编号（1, 2, 3, 4）
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam PKT_HEAD_1   = 8'h04;          // 包头1
localparam PKT_HEAD_2   = 8'h0C;          // 包头2
localparam PKT_LEN      = 4'd12;          // 数据包长度（字节）

parameter STATE_0 = 3'd0;                           
parameter STATE_1 = 3'd1;                                                                           
parameter STATE_2 = 3'd2;                                                                           
parameter STATE_3 = 3'd3;
parameter STATE_4 = 3'd4;
parameter STATE_5 = 3'd5;

reg          [2:0]  state, next_state; 
reg                 flag_0_to_1;
reg                 flag_1_to_2;
reg                 flag_2_to_3;
reg                 flag_3_to_4;
reg                 flag_4_to_5;
reg                 flag_5_to_0;

reg [3:0]   cnt;                        // 延迟计数标志
reg [3:0]   rx_cnt;                     // 接收字节计数
reg [7:0]   rx_buffer [0:11];            // 接收数据缓冲区

wire [7:0]  uart_rx_data;               // UART接收数据
wire        uart_rx_flag;               // UART接收完成标志

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

uart_rx #(
    .UART_BPS(UART_BPS),
    .CLK_FREQ(CLK_FREQ)
) u_uart_rx (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .rx(rx),
    .po_data(uart_rx_data),
    .po_flag(uart_rx_flag)
);

//********************************************************************//
//***************************** 接收数据 *****************************//
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
        cnt <= 4'd0;
        rx_cnt <= 4'd0;
        match_id <= 0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_0 <= 0;
    end
    else begin
        case (state)
            STATE_0: begin
                flag_5_to_0 <= 0;  
                rx_cnt <= 4'd0;
                
                if(uart_rx_flag) begin
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin  
                flag_0_to_1 <= 0; 
                
                if (uart_rx_data == PKT_HEAD_1) begin
                    rx_buffer[0] <= uart_rx_data;
                    rx_cnt <= 1;
                    
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end 
            STATE_2: begin
                flag_1_to_2 <= 0;
                    
                if(uart_rx_flag) begin
                    
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end
            end
            STATE_3: begin  
                flag_2_to_3 <= 0; 
                
                if (uart_rx_data == PKT_HEAD_2) begin
                    rx_buffer[1] <= uart_rx_data;
                    rx_cnt <= 2;
                    
                    flag_3_to_4 <= 1;
                end
                else begin
                    flag_3_to_4 <= 0;
                end
            end 
            STATE_4: begin
                flag_3_to_4 <= 0;
                
                if(uart_rx_flag) begin
                    cnt <= cnt + 1;
                end
                
                if(cnt) begin
                    cnt <= 0;
                    
                    rx_buffer[rx_cnt] <= uart_rx_data;
                    rx_cnt <= rx_cnt + 1'b1;
                end
                if (rx_cnt == PKT_LEN) begin
                    flag_4_to_5 <= 1;
                end
                else begin
                    flag_4_to_5 <= 0;
                end               
            end
            STATE_5: begin
                flag_4_to_5 <= 0;
                
                if (rx_buffer[0] == PKT_HEAD_1 && rx_buffer[1] == PKT_HEAD_2) begin
                
                    if(rx_buffer[11] == 8'h74) begin
                        match_id <= 1;
                    end
                    else if(rx_buffer[11] == 8'h0B) begin
                        match_id <= 2;
                    end
                    else if(rx_buffer[11] == 8'h6D) begin
                        match_id <= 3;
                    end
                    else if(rx_buffer[11] == 8'h15) begin
                        match_id <= 4;
                    end
                    else begin
                        match_id <= 0;
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
