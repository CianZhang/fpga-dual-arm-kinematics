module uart_tx_top_test (
    input  wire           sys_clk,     // 50MHz系统时钟
    input  wire           sys_rst_n,   // 低电平有效复位
    
    output wire           tx            // UART发送输出
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

reg signed   [15:0] data1;
reg signed   [15:0] data2; 
reg signed   [15:0] data3;

reg          [31:0] cnt;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

uart_tx_top uart_tx_top_inst (
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),
    .data1(data1),    
    .data2(data2),    
    .data3(data3),    
    .tx(tx)        
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        data1 <= 0;
        data2 <= 0;
        data3 <= 0;
        cnt <= 0;
    end
    else begin
        data1 <= 600;
        data2 <= -200;
        data3 <= -4800;
        cnt <= cnt + 1;
        if(cnt > 5000000) begin
            data1 <= 600;
            data2 <= 0;
            data3 <= 0;  
        end          
    end
end

endmodule
