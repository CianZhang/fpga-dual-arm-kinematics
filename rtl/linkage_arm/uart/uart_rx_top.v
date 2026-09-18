module uart_rx_top
#(
    parameter UART_BPS     = 'd9600,         // 串口波特率
    parameter CLK_FREQ     = 'd50_000_000    // 时钟频率
)
(
    input  wire              sys_clk,        // 系统时钟50MHz
    input  wire              sys_rst_n,      // 全局复位，低有效
    input  wire              rx,             // 串口接收数据

    output reg signed [7:0]  rx_data1,       // 接收数据1，8位宽接收电机序号
    output reg signed [7:0]  rx_data2,       // 接收数据2，8位宽接收旋转方向
    output reg signed [31:0] rx_data3,       // 接收数据3，32位宽接收电机角度
    
    output reg               rx_done         // 接收完成标志
);

//********************************************************************//
//****************** Parameter and Internal Signal ********************//
//********************************************************************//

// 参数定义
localparam PKT_HEAD   = 8'hAA;          // 包头
localparam PKT_TAIL   = 8'h55;          // 包尾
localparam PKT_LEN    = 4'd8;           // 数据包长度（字节）

parameter STATE_0 = 2'b00;                           
parameter STATE_1 = 2'b01;                                                                           
parameter STATE_2 = 2'b10;                                                                           
parameter STATE_3 = 2'b11;

reg          [1:0]  state, next_state; 
reg                 flag_0_to_1;
reg                 flag_1_to_2;
reg                 flag_2_to_3;
reg                 flag_3_to_0;

reg [3:0]   cnt;                        // 延迟计数标志
reg [3:0]   rx_cnt;                     // 接收字节计数
reg [7:0]   rx_buffer [0:7];            // 接收数据缓冲区

wire [7:0]  uart_rx_data;               // UART接收数据
wire        uart_rx_flag;               // UART接收完成标志

wire [7:0]  rx_buffer0;
wire [7:0]  rx_buffer1;
wire [7:0]  rx_buffer2;
wire [7:0]  rx_buffer3;
wire [7:0]  rx_buffer4;
wire [7:0]  rx_buffer5;
wire [7:0]  rx_buffer6;
wire [7:0]  rx_buffer7;
assign rx_buffer0 = rx_buffer[0];
assign rx_buffer1 = rx_buffer[1];
assign rx_buffer2 = rx_buffer[2];
assign rx_buffer3 = rx_buffer[3];
assign rx_buffer4 = rx_buffer[4];
assign rx_buffer5 = rx_buffer[5];
assign rx_buffer6 = rx_buffer[6];
assign rx_buffer7 = rx_buffer[7];

//********************************************************************//
//************************* Instantiation ****************************//
//********************************************************************//

// 实例化UART接收模块
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
            if(flag_2_to_3 == 1)
                next_state = STATE_3; // 如果flag_2_to_3 == 3，转到STATE_3
            else 
                next_state = STATE_2; // 否则保持在STATE_2
        end
        STATE_3: begin
            if(flag_3_to_0 == 1)
                next_state = STATE_0; // 如果flag_3_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_3; // 否则保持在STATE_3
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end 

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cnt <= 4'd0;
        rx_cnt <= 4'd0;
        rx_done <= 1'b0;
        rx_data1 <= 16'd0;
        rx_data2 <= 16'd0;
        rx_data3 <= 16'd0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_0 <= 0;
    end
    else begin
        case (state)
            STATE_0: begin
                flag_3_to_0 <= 0;  
                rx_cnt <= 4'd0;
                
                if(uart_rx_flag) begin
                    rx_done <= 1'b0;
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin  
                flag_0_to_1 <= 0; 
                
                if (uart_rx_data == PKT_HEAD) begin
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
                    cnt <= cnt + 1;
                end
                
                if(cnt) begin
                    cnt <= 0;
                    
                    rx_buffer[rx_cnt] <= uart_rx_data;
                    rx_cnt <= rx_cnt + 1'b1;
                end
                if (rx_cnt == PKT_LEN) begin
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end               
            end
            STATE_3: begin
                flag_2_to_3 <= 0;
                
                if (rx_buffer[0] == PKT_HEAD && rx_buffer[7] == PKT_TAIL) begin
                    rx_data1 <= rx_buffer[1];
                    rx_data2 <= rx_buffer[2];
                    rx_data3 <= {rx_buffer[3], rx_buffer[4], rx_buffer[5], rx_buffer[6]};
                    
                    rx_done <= 1'b1;
                    
                    flag_3_to_0 <= 1;
                end
                else begin
                    flag_3_to_0 <= 0;
                end
            end                
            default: ; 
        endcase
    end
end 

endmodule
