module key (
    input   wire            clk,
    input   wire            rstn,
    input   wire            key_in,
    output  reg             key_out
);
parameter KEY_CNT = 19'd499_999;
reg     [18:0]  cnt;

always @(posedge clk or negedge rstn) begin
    if(rstn == 1'b0)
        cnt <= 19'd0;
    else if(key_in == 1'b1)
        cnt <= 19'd0;
    else if(cnt == KEY_CNT)
        cnt <= cnt;
    else
        cnt <= cnt + 1'b1;
end

always @(posedge clk or negedge rstn) begin
    if(rstn == 1'b0)
        key_out <= 1'b0;
    else if(cnt == KEY_CNT - 1)
        key_out <= 1'b1;
    else
        key_out <= 1'b0;
end

endmodule