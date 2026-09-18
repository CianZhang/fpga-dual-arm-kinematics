`timescale 1ns / 1ps

module tb_signed_divider;

  // Clock and reset
  reg clk, rst;
  always begin
    clk = 0;
    forever #5 clk = ~clk; // 10ns period
  end
  initial begin
    rst = 0;
    @(posedge clk); #1;
    rst = 1;
    @(posedge clk); #1;
    rst = 0;
  end

  // Parameters and signals
  localparam WIDTH = 32;
  reg start;
  reg  [WIDTH-1:0] dividend;
  reg  [WIDTH-1:0] divisor;
  wire [WIDTH-1:0] quotient;
  wire [WIDTH-1:0] remainder;
  wire valid, zeroErr;

  // Reference results for verification
  wire [WIDTH-1:0] quotient_ref  = $signed(dividend) / $signed(divisor);
  wire [WIDTH-1:0] remainder_ref = $signed(dividend) % $signed(divisor);
  wire error = valid && (quotient_ref != quotient || remainder_ref != remainder) && !(zeroErr && divisor == 0);

  // Instantiate the signed divider
  signed_divider #(
    .WIDTH(WIDTH),
    .CACHING(0),
    .INIT_VLD(0)
  ) uut (
    .clk(clk),
    .rst(rst),
    .start(start),
    .dividend(dividend),
    .divisor(divisor),
    .quotient(quotient),
    .remainder(remainder),
    .zeroErr(zeroErr),
    .valid(valid)
  );

  // Test procedure
  initial begin
    // Initialize inputs
    // dividend =  32'd0 ;
    // divisor =   32'd0  ;
    // start = 1'd0;
    {start,dividend,divisor} = {1'd0,32'd0,32'd0};
    repeat(5) @(posedge clk); #1;

    // Test 1: Positive / Positive
    $display("Test 1: 100 / 5");
    {start,dividend,divisor} = {1'd1,32'd47360,32'd300};

    // dividend =  32'd47360 ;
    // divisor =   32'd300   ;
    // start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    // Test 2: Negative / Positive
    $display("Test 2: -100 / 5");
    dividend = 32'd2756096;
    divisor = 32'd45000;
    start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    // Test 3: Positive / Negative
    $display("Test 3: 100 / -5");
    dividend = 32'd181248;
    divisor = 32'd55500;
    start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    // Test 4: Negative / Negative
    $display("Test 4: -100 / -5");
    dividend = -32'd100;
    divisor = -32'd5;
    start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    // Test 5: Divide by zero
    $display("Test 5: 100 / 0");
    dividend = 32'd100;
    divisor = 32'd0;
    start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    // Test 6: Large numbers
    $display("Test 6: 2^50 / 2^10");
    dividend = 32'd1 << 50;
    divisor = 32'd1 << 10;
    start = 1;
    @(posedge clk); #1;
    start = 0;
    while (!valid) @(posedge clk);
    #1; $display("Result: quotient = %d, remainder = %d, zeroErr = %b, error = %b", 
                 $signed(quotient), $signed(remainder), zeroErr, error);
    repeat(5) @(posedge clk); #1;

    $stop;
  end

endmodule