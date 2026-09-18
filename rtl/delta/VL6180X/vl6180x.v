module vl6180x (
    input   wire            clk,
    input   wire            rstn,

    input   wire            start_flag, // 给一个脉冲
    output  reg     [7:0]   rddata,     // 读取到的数据
    output  wire            rd_flag,    // 读取标志脉冲

    output  wire            i2c_scl,
    inout   wire            i2c_sda
);

parameter S_INIT = 1'b0;
parameter S_DATA = 1'b1;

parameter SD_READ0  = 3'd0;
parameter SD_WRITE0 = 3'd1;
parameter SD_READ1  = 3'd2;
parameter SD_READD  = 3'd3;
parameter SD_WRITE1 = 3'd4;
parameter SD_WAIT   = 3'd5;

//parameter WAIT_COUNT = 15'd24_999;
//parameter WAIT_COUNT = 16'd49_999;      // 50ms读取一次
parameter WAIT_COUNT = 16'd100000;      // 50ms读取一次

parameter INIT_NUM = 6'd39;

reg             state;
reg     [2:0]   state_data;
wire            i2c_wr;
wire            i2c_rd;
reg             i2c_start;
reg             i2c_start_flag;
reg     [7:0]   i2c_wrdata;
wire    [7:0]   i2c_rddata;
reg     [15:0]  i2c_addr;
reg     [5:0]   init_cnt;
wire            i2c_clk;
reg             i2c_clk_reg;
wire            i2c_clk_up;
wire            i2c_end;
reg             i2c_end_reg;
wire            end_flag;
reg     [14:0]   wait_cnt;

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        wait_cnt <= 15'd0;
    else if(state_data == SD_WAIT)
        wait_cnt <= wait_cnt + 1'b1;
    else
        wait_cnt <= 15'd0;
end

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        state <= S_INIT;
    else if(init_cnt == INIT_NUM + 1)
        state <= S_DATA;
end

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        state_data <= SD_READ0;
    else if(state == S_DATA) begin
        if((state_data == SD_READ0) && (i2c_rddata[0] == 1'b1) && (i2c_end))
            state_data <= SD_WRITE0;
        else if((state_data == SD_WRITE0) && (i2c_end))
            state_data <= SD_READ1;
        else if((state_data == SD_READ1) && (i2c_rddata[2] == 1'b1) && (i2c_end))
            state_data <= SD_READD;
        else if((state_data == SD_READD) && (i2c_end))
            state_data <= SD_WRITE1;
        else if((state_data == SD_WRITE1) && (i2c_end))
            state_data <= SD_WAIT;
        else if((state_data == SD_WAIT) && (wait_cnt == WAIT_COUNT))
            state_data <= SD_READ0;
    end
    else
        state_data <= SD_READ0;
end

assign i2c_wr = ((state == S_INIT) || (state_data == SD_WRITE0) || (state_data == SD_WRITE1));

assign i2c_rd = ((state == S_DATA) && ((state_data == SD_READ0) || (state_data == SD_READ1) || (state_data == SD_READD)));

always @(*) begin
    if(state == S_INIT)
        case (init_cnt)
            6'd1    : begin i2c_addr = 16'h0207; i2c_wrdata = 8'h01; end
            6'd2    : begin i2c_addr = 16'h0208; i2c_wrdata = 8'h01; end
            6'd3    : begin i2c_addr = 16'h0096; i2c_wrdata = 8'h00; end
            6'd4    : begin i2c_addr = 16'h0097; i2c_wrdata = 8'hfd; end
            6'd5    : begin i2c_addr = 16'h00e3; i2c_wrdata = 8'h00; end
            6'd6    : begin i2c_addr = 16'h00e4; i2c_wrdata = 8'h04; end
            6'd7    : begin i2c_addr = 16'h00e5; i2c_wrdata = 8'h02; end
            6'd8    : begin i2c_addr = 16'h00e6; i2c_wrdata = 8'h01; end
            6'd9    : begin i2c_addr = 16'h00e7; i2c_wrdata = 8'h03; end
            6'd10   : begin i2c_addr = 16'h00f5; i2c_wrdata = 8'h02; end
            6'd11   : begin i2c_addr = 16'h00d9; i2c_wrdata = 8'h05; end
            6'd12   : begin i2c_addr = 16'h00db; i2c_wrdata = 8'hce; end
            6'd13   : begin i2c_addr = 16'h00dc; i2c_wrdata = 8'h03; end
            6'd14   : begin i2c_addr = 16'h00dd; i2c_wrdata = 8'hf8; end
            6'd15   : begin i2c_addr = 16'h009f; i2c_wrdata = 8'h00; end
            6'd16   : begin i2c_addr = 16'h00a3; i2c_wrdata = 8'h3c; end
            6'd17   : begin i2c_addr = 16'h00b7; i2c_wrdata = 8'h00; end
            6'd18   : begin i2c_addr = 16'h00bb; i2c_wrdata = 8'h3c; end
            6'd19   : begin i2c_addr = 16'h00b2; i2c_wrdata = 8'h09; end
            6'd20   : begin i2c_addr = 16'h00ca; i2c_wrdata = 8'h09; end
            6'd21   : begin i2c_addr = 16'h0198; i2c_wrdata = 8'h01; end
            6'd22   : begin i2c_addr = 16'h01b0; i2c_wrdata = 8'h17; end
            6'd23   : begin i2c_addr = 16'h01ad; i2c_wrdata = 8'h00; end
            6'd24   : begin i2c_addr = 16'h00ff; i2c_wrdata = 8'h05; end
            6'd25   : begin i2c_addr = 16'h0100; i2c_wrdata = 8'h05; end
            6'd26   : begin i2c_addr = 16'h0199; i2c_wrdata = 8'h05; end
            6'd27   : begin i2c_addr = 16'h01a6; i2c_wrdata = 8'h1b; end
            6'd28   : begin i2c_addr = 16'h01ac; i2c_wrdata = 8'h3e; end
            6'd29   : begin i2c_addr = 16'h01a7; i2c_wrdata = 8'h1f; end
            6'd30   : begin i2c_addr = 16'h0030; i2c_wrdata = 8'h00; end
            6'd31   : begin i2c_addr = 16'h0011; i2c_wrdata = 8'h10; end
            6'd32   : begin i2c_addr = 16'h010a; i2c_wrdata = 8'h30; end
            6'd33   : begin i2c_addr = 16'h003f; i2c_wrdata = 8'h46; end
            6'd34   : begin i2c_addr = 16'h0031; i2c_wrdata = 8'hff; end
            6'd35   : begin i2c_addr = 16'h0040; i2c_wrdata = 8'h63; end
            6'd36   : begin i2c_addr = 16'h002e; i2c_wrdata = 8'h01; end
            6'd37   : begin i2c_addr = 16'h001b; i2c_wrdata = 8'h09; end
            6'd38   : begin i2c_addr = 16'h003e; i2c_wrdata = 8'h31; end
            6'd39   : begin i2c_addr = 16'h0014; i2c_wrdata = 8'h24; end
            default : begin i2c_addr = 16'h0000; i2c_wrdata = 8'h00; end
        endcase
    else if(state == S_DATA)
        case (state_data)
            SD_READ0 : begin i2c_addr = 16'h004d; i2c_wrdata = 8'h00; end
            SD_WRITE0: begin i2c_addr = 16'h0018; i2c_wrdata = 8'h01; end
            SD_READ1 : begin i2c_addr = 16'h004f; i2c_wrdata = 8'h00; end
            SD_READD : begin i2c_addr = 16'h0062; i2c_wrdata = 8'h00; end
            SD_WRITE1: begin i2c_addr = 16'h0015; i2c_wrdata = 8'h07; end
            default  : begin i2c_addr = 16'h0000; i2c_wrdata = 8'h00; end
        endcase
    else
        begin i2c_addr = 16'h0000; i2c_wrdata = 8'h00; end
end

always @(posedge clk or negedge rstn) begin
    if(rstn == 1'b0)
        i2c_start_flag <= 1'b0;
    else if(start_flag == 1'b1)
        i2c_start_flag <= 1'b1;
    else if(i2c_clk_up == 1'b1)
        i2c_start_flag <= 1'b0;
end

always @(posedge clk or negedge rstn) begin
    if(rstn == 1'b0)
        i2c_clk_reg <= 1'b0;
    else
        i2c_clk_reg <= i2c_clk;
end

assign i2c_clk_up = i2c_clk & (~i2c_clk_reg);

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        init_cnt <= 6'd0;
    else if((init_cnt == 6'd0) && (i2c_start_flag))
        init_cnt <= 6'd1;
    else if((init_cnt != 6'd0) && (init_cnt != INIT_NUM + 1) && (i2c_end))
        init_cnt <= init_cnt + 1'b1;
end

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        i2c_start <= 1'b0;
    else if((state_data != SD_WRITE1) && (i2c_start_flag || i2c_end))
        i2c_start <= 1'b1;
    else if(wait_cnt == WAIT_COUNT)
        i2c_start <= 1'b1;
    else
        i2c_start <= 1'b0;
end

always @(posedge i2c_clk or negedge rstn) begin
    if(rstn == 1'b0)
        rddata <= 8'd0;
    else if((state_data == SD_READD) && (i2c_end))
        rddata <= i2c_rddata;
end

always @(posedge clk or negedge rstn) begin
    if(rstn == 1'b0)
        i2c_end_reg <= 1'b0;
    else
        i2c_end_reg <= i2c_end;
end

assign end_flag = i2c_end_reg & (~i2c_end);
assign rd_flag = (state_data == SD_WRITE1)? end_flag:1'b0;

i2c_ctrl i2c_ctrl_inst (
    .sys_clk    (clk),
    .sys_rst_n  (rstn),
    .wr_en      (i2c_wr),
    .rd_en      (i2c_rd),
    .i2c_start  (i2c_start),
    .addr_num   (1'b1),
    .byte_addr  (i2c_addr),
    .wr_data    (i2c_wrdata),

    .i2c_clk    (i2c_clk),
    .i2c_end    (i2c_end),
    .rd_data    (i2c_rddata),
    .i2c_scl    (i2c_scl),
    .i2c_sda    (i2c_sda)
);

endmodule