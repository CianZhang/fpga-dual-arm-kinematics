`timescale 1ns / 1ps
module signed_divider #(
  parameter WIDTH    = 32,
  parameter CACHING  =  0,
  parameter INIT_VLD =  0
)(
  input clk, 
  input rst, 
  input start, 
  input      [WIDTH-1:0] dividend, 
  input      [WIDTH-1:0] divisor, 
  output reg [WIDTH-1:0] quotient, 
  output reg [WIDTH-1:0] remainder, 
  output reg zeroErr, 
  output reg valid,
  output reg overflow
);

  wire dividend_sign = dividend[WIDTH-1];
  wire divisor_sign  = divisor[WIDTH-1];
  wire quotient_sign = dividend_sign ^ divisor_sign;
  wire remainder_sign = dividend_sign;

  wire [WIDTH-1:0] dividend_abs = dividend_sign ? (~dividend + 1) : dividend;
  wire [WIDTH-1:0] divisor_abs  = divisor_sign  ? (~divisor + 1)  : divisor;

  wire [WIDTH-1:0] quotient_uns;
  wire [WIDTH-1:0] remainder_uns;
  wire zeroErr_uns;
  wire valid_uns;

  reg buffer;

always @(posedge clk) begin
  if (rst) 
      buffer <= 1'b0; 
  else
      buffer <= start; 
end
  
wire start_to_divider = ~buffer & start;

  divider #(
    .WIDTH(WIDTH),
    .CACHING(CACHING),
    .INIT_VLD(INIT_VLD)
  ) unsigned_divider (
    .clk(clk),
    .rst(rst),
    .start(start_to_divider),
    .dividend(dividend_abs),
    .divisor(divisor_abs),
    .quotient(quotient_uns),
    .remainder(remainder_uns),
    .zeroErr(zeroErr_uns),
    .valid(valid_uns)
  );

  // Overflow detection: -2^63 / -1 case
  wire overflow_condition = (dividend == {1'b1, {WIDTH-1{1'b0}}}) && (divisor == {WIDTH{1'b1}});

  always @(posedge clk) begin
    if (rst) begin
      quotient  <= 0;
      remainder <= 0;
      zeroErr   <= 0;
      valid     <= 0;
      overflow  <= 0;
    end else if (valid_uns) begin
      valid <= 1;
      zeroErr <= zeroErr_uns;
      overflow <= overflow_condition;
      
      if (zeroErr_uns) begin
        quotient  <= 0;  // Clear outputs on zero divide
        remainder <= 0;
      end else if (overflow_condition) begin
        quotient  <= 0;  // Indicate overflow with special value
        remainder <= 0;
      end else begin
        quotient  <= quotient_sign ? (~quotient_uns + 1) : quotient_uns;
        remainder <= remainder_sign ? (~remainder_uns + 1) : remainder_uns;
      end
    end else begin
      valid <= 0;
    end
  end

endmodule