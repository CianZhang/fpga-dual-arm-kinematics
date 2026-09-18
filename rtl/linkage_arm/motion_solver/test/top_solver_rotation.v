module top_solver_rotation
(
    input wire                  sys_clk,      
    input wire                  sys_rst_n, 
    
    output wire                 step_pulse,
    output wire                 direction,
    output wire                 reached_target
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
 
reg signed [15:0]    x;                      // 空间坐标X     
reg signed [15:0]    y;                      // 空间坐标Y

reg                  turn_direction;         // 旋转方向

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

solver_rotation solver_rotation_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .i_x(x),           
    .i_y(y),           
    .turn_direction(turn_direction),
    .step_pulse(step_pulse),
    .direction(direction),
    .reached_target(reached_target)
);

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        x <= 0;
        y <= 0;
        turn_direction <= 0;
    end
    else begin
        x <= 0;
        y <= 100;
        turn_direction <= 1;
    end 
end 
   
endmodule
  