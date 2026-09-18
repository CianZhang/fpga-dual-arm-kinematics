module tset_RFID_uart (
    input  wire        sys_clk,      // 系统时钟 50MHz
    input  wire        sys_rst_n,    // 全局复位，低电平有效
    
    input  wire        rx,           // 串口接收引脚
    output wire        tx,           // 串口发送引脚
    
    output wire [2:0]  match_id,     // 输出物品编号（1, 2, 3, 4）
    output wire        data_valid    // 数据有效信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

parameter MAX_COUNT = 50_000_000; // 1秒计数目标

reg         send_trig;
reg  [31:0] counter;

parameter STATE_0  = 2'd0;                           
parameter STATE_1  = 2'd1;                
parameter STATE_2  = 2'd2;                      
parameter STATE_3  = 2'd3;            

reg  [1:0]  state, next_state; 
reg         flag_0_to_1;
reg         flag_1_to_2; 
reg         flag_2_to_3; 
reg         flag_3_to_0;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

RFID_uart RFID_uart_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .send_trig(send_trig),
    .rx(rx),       
    .tx(tx),       
    .match_id(match_id), 
    .data_valid(data_valid)
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
        counter <= 0;
        send_trig <= 0;
        
        flag_0_to_1 <= 1'b0;
        flag_1_to_2 <= 1'b0; 
        flag_2_to_3 <= 1'b0; 
        flag_3_to_0 <= 1'b0;
    end
    else begin
        case(state)
            STATE_0: begin
                flag_3_to_0 <= 1'b0;
                
                if (counter == MAX_COUNT - 1) begin
                    counter <= 0;
                    
                    flag_0_to_1 <= 1;
                end
                else begin
                    counter <= counter + 1;
                    
                    flag_0_to_1 <= 0;
                end
            end                
            STATE_1: begin
                flag_0_to_1 <= 0;
                
                send_trig <= 1;  
                    
                if(!data_valid) begin 
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end            
            STATE_2: begin
                flag_1_to_2 <= 0;
                
                send_trig <= 0;

                if(data_valid) begin 
                    flag_2_to_3 <= 1;
                end
                else begin
                    flag_2_to_3 <= 0;
                end
            end                             
            STATE_3: begin
                flag_2_to_3 <= 0;
                
                send_trig <= 0;

                if(data_valid) begin 
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
