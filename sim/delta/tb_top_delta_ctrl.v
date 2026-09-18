`timescale 1ns / 1ns

module tb_top_delta_ctrl;

// 测试信号定义
reg         sys_clk;             // 系统时钟，50MHz
reg         sys_rst_n;           // 系统复位
    
reg         start_inst;          // 启动初始化机械臂
reg         start_move;          // 启动运动信号

reg         limit_switch_1;         
reg         limit_switch_2;         
reg         limit_switch_3;

wire        Delta_step_pulse_1;
wire        Delta_direction_1;
wire        Delta_step_pulse_2;
wire        Delta_direction_2;
wire        Delta_step_pulse_3;
wire        Delta_direction_3;
wire        step_pulse_1;   
wire        direction_1;    
wire        step_pulse_2;   
wire        direction_2;    
wire        step_pulse_3;   
wire        direction_3;    
wire        step_pulse_4;   
wire        direction_4;    
wire        step_pulse_tran;
wire        direction_tran; 
wire        electromagnet;  
wire        sucker_pump;    
wire        sucker_valve;   
wire        tx;
reg         rx;

top_delta_ctrl top_delta_ctrl_inst(
    .sys_clk(sys_clk),                
    .sys_rst_n(sys_rst_n),              
    .start_inst(start_inst),             
    .start_move(start_move),             
    .limit_switch_1(limit_switch_1),         
    .limit_switch_2(limit_switch_2),         
    .limit_switch_3(limit_switch_3),         
    .Delta_step_pulse_1(Delta_step_pulse_1_ctrl),
    .Delta_direction_1(Delta_direction_1_ctrl),
    .Delta_step_pulse_2(Delta_step_pulse_2_ctrl),
    .Delta_direction_2(Delta_direction_2_ctrl),
    .Delta_step_pulse_3(Delta_step_pulse_3_ctrl),
    .Delta_direction_3(Delta_direction_3_ctrl),
    .step_pulse_1(step_pulse_1),           
    .direction_1(direction_1),                  
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4), 
    .step_pulse_tran(step_pulse_tran),    
    .direction_tran(direction_tran),     
    .electromagnet(electromagnet),      
    .sucker_pump(sucker_pump),        
    .sucker_valve(sucker_valve),       
    .rx(rx),            
    .tx(tx)             
);

// ******************************************************************** //
// ********************* Clock Generation **************************** //
// ******************************************************************** //

GTP_GRS GRS_INST(
    .GRS_N(1'b1)
    );

initial begin
    sys_clk = 0;
    start_inst = 0;
    start_move = 0;
end

always #10 sys_clk = ~sys_clk;

// ******************************************************************** //
// ************************ Testbench Code **************************** //
// ******************************************************************** //

// 测试序列
initial begin
    sys_rst_n = 0;                  // 复位信号初始化为低

    #100;                           // 等待100ns

    sys_rst_n = 1;                  // 释放复位信号

    #200; 
    start_inst = 1;

    #62500;
    
    start_move = 1;

    #62500000;   
    #62500000;
    #62500000;

    $stop;
end

endmodule
