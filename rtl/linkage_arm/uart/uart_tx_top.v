module uart_tx_top (
    input  wire            sys_clk,     // 50MHz系统时钟
    input  wire            sys_rst_n,   // 低电平有效复位
    input  signed [15:0]   data1,       // 第一个 16 位有符号数
    input  signed [15:0]   data2,       // 第二个 16 位有符号数
    input  signed [15:0]   data3,       // 第三个 16 位有符号数    
    
    output wire            tx           // UART发送输出
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

localparam UART_BPS    = 9600;          // 波特率
localparam CLK_FREQ    = 50;            // 时钟频率
localparam TIME_50MS   = 2500000;       // 50ms计数（2,500,000周期）

wire                tx_done;            // UART发送完成

reg signed   [15:0] data1_reg;
reg signed   [15:0] data2_reg; 
reg signed   [15:0] data3_reg; 

reg          [7:0]  pi_data;            // 发送数据
reg                 pi_flag;            // 数据有效标志

parameter STATE_0 = 4'b0000;
parameter STATE_1 = 4'b0001;       
parameter STATE_2 = 4'b0010;       
parameter STATE_3 = 4'b0011;       
parameter STATE_4 = 4'b0100;       
parameter STATE_5 = 4'b0101;       
parameter STATE_6 = 4'b0110;       
parameter STATE_7 = 4'b0111;       
parameter STATE_8 = 4'b1000;       

reg          [3:0]  state, next_state; 
reg                 flag_0_to_1;
reg                 flag_1_to_2;
reg                 flag_2_to_3;
reg                 flag_3_to_4;
reg                 flag_4_to_5;
reg                 flag_5_to_6;
reg                 flag_6_to_7;
reg                 flag_7_to_8;
reg                 flag_8_to_0;

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
            if(flag_7_to_8 == 1)
                next_state = STATE_8; // 如果flag_7_to_8 == 1，转到STATE_8
            else 
                next_state = STATE_7; // 否则保持在STATE_7
        end       
        STATE_8: begin
            if(flag_8_to_0 == 1)
                next_state = STATE_0; // 如果flag_8_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_8; // 否则保持在STATE_8
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pi_data <= 0;
        pi_flag <= 0;
        data1_reg <= 0;
        data2_reg <= 0;
        data3_reg <= 0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
        flag_5_to_6 <= 0;
        flag_6_to_7 <= 0;
        flag_7_to_8 <= 0;
        flag_8_to_0 <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                flag_8_to_0 <= 0;
                pi_flag <= 0;
                
                if((data1 != data1_reg) || (data2 != data2_reg) || (data3 != data3_reg)) begin
                    data1_reg <= data1;
                    data2_reg <= data2;
                    data3_reg <= data3;

                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end    
            STATE_1: begin
                flag_0_to_1 <= 0;
                
                pi_data <= 8'hAA;
                pi_flag <= 1;
                
                flag_1_to_2 <= 1;
            end                        
            STATE_2: begin
                flag_1_to_2 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data1[7:0];
                    pi_flag <= 1; 
                    
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end
            end 
            STATE_3: begin
                flag_2_to_3 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data1[15:8];
                    pi_flag <= 1; 
                    
                    flag_3_to_4 <= 1;
                end
                else begin
                    flag_3_to_4 <= 0;
                end
            end 
            STATE_4: begin
                flag_3_to_4 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data2[7:0];
                    pi_flag <= 1; 
                    
                    flag_4_to_5 <= 1;
                end
                else begin
                    flag_4_to_5 <= 0;
                end
            end 
            STATE_5: begin
                flag_4_to_5 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data2[15:8];
                    pi_flag <= 1; 
                    
                    flag_5_to_6 <= 1;
                end
                else begin
                    flag_5_to_6 <= 0;
                end
            end 
            STATE_6: begin
                flag_5_to_6 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data3[7:0];
                    pi_flag <= 1; 
                    
                    flag_6_to_7 <= 1;
                end
                else begin
                    flag_6_to_7 <= 0;
                end
            end 
            STATE_7: begin
                flag_6_to_7 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= data3[15:8];
                    pi_flag <= 1; 
                    
                    flag_7_to_8 <= 1;
                end
                else begin
                    flag_7_to_8 <= 0;
                end
            end
            STATE_8: begin
                flag_7_to_8 <= 0;
                
                pi_flag <= 0;
                
                if(tx_done) begin   
                    pi_data <= 8'h55;
                    pi_flag <= 1; 
                    
                    flag_8_to_0 <= 1;
                end
                else begin
                    flag_8_to_0 <= 0;
                end
            end
            default: ; 
        endcase
    end
end            

endmodule
