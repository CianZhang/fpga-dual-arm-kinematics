module vl6180x_top (
    input   wire            sys_clk,
    input   wire            sys_rst_n,
    input   wire            key,

    output  wire            i2c_scl,
    inout   wire            i2c_sda,
    
    output  wire             tx           // 串口发送数据线
);

//********************************************************************//
//****************** Internal Signal and Parameter *******************//
//********************************************************************//

reg    [15:0]  data2;
reg    [15:0]  data3;

wire   [7:0]   rddata;
wire           key_flag;
reg            start_flag; 
reg            rd_flag;

//********************************************************************//
//************************** Main Code *******************************//
//********************************************************************//

key key_inst (
    .clk(sys_clk),
    .rstn(sys_rst_n),
    .key_in(key),
    .key_out(key_flag)
);

vl6180x vl6180x_inst(
    .clk(sys_clk),
    .rstn(sys_rst_n),
    .start_flag(start_flag),
    .rddata(rddata),
    .rd_flag(rd_flag),
    .i2c_scl(i2c_scl),
    .i2c_sda(i2c_sda)
);

uart_tx_top uart_tx_top_inst (
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),
    .data1(rddata),    
    .data2(data2),    
    .data3(data3),    
    .tx(tx)        
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge clk) begin
    start_flag <= key_flag;
end

endmodule      
