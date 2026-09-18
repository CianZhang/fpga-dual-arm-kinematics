module top_delta_ctrl
(
    input  wire                 sys_clk,                // 系统时钟，50MHz
    input  wire                 sys_rst_n,              // 系统复位，低电平有效
    
    input  wire                 start_inst,             // 启动初始化机械臂
    input  wire                 start_move,             // 启动运动信号
    
    input  wire                 limit_switch_1,         // 电机1限位开关信号
    input  wire                 limit_switch_2,         // 电机2限位开关信号
    input  wire                 limit_switch_3,         // 电机3限位开关信号

    output wire                 Delta_step_pulse_1,     // Delta机械臂电机 1 脉冲信号
    output wire                 Delta_direction_1,      // Delta机械臂电机 1 方向信号
    output wire                 Delta_step_pulse_2,     // Delta机械臂电机 2 脉冲信号
    output wire                 Delta_direction_2,      // Delta机械臂电机 2 方向信号
    output wire                 Delta_step_pulse_3,     // Delta机械臂电机 3 脉冲信号
    output wire                 Delta_direction_3,      // Delta机械臂电机 3 方向信号
 
    output wire                 step_pulse_1,           // 平移步进电机脉冲信号
    output wire                 direction_1,            // 平移步进电机方向信号 
    output wire                 step_pulse_2,           // 旋转步进电机脉冲信号
    output wire                 direction_2,            // 旋转步进电机方向信号   
    output wire                 step_pulse_3,           // 大臂步进电机脉冲信号
    output wire                 direction_3,            // 大臂步进电机方向信号
    output wire                 step_pulse_4,           // 小臂步进电机脉冲信号
    output wire                 direction_4,            // 小臂步进电机方向信号

    output wire                 step_pulse_tran,        // 传输带电机脉冲信号
    output wire                 direction_tran,         // 传输带步进电机方向信号 
 
    output wire                 electromagnet,          // 电磁铁打开信号，高电平有效    
    output wire                 sucker_pump,            // 吸盘的泵控制信号
    output wire                 sucker_valve,           // 吸盘的电磁阀控制信号
    
    output wire                 tx,                     // UART发送输出
    input  wire                 rx                      // 串口接收数据    
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
 
wire              reached_target_tran;
reg               data_valid;

reg        [3:0]  ltem_red_1;             // 红色六边形
reg        [3:0]  ltem_red_2;             // 红色圆形
reg        [3:0]  ltem_red_3;             // 红色正方形
reg        [3:0]  ltem_green_1;           // 绿色六边形
reg        [3:0]  ltem_green_2;           // 绿色圆形
reg        [3:0]  ltem_green_3;           // 绿色正方形
reg        [3:0]  ltem_blue_1;            // 蓝色六边形
reg        [3:0]  ltem_blue_2;            // 蓝色圆形
reg        [3:0]  ltem_blue_3;            // 蓝色正方形
reg        [3:0]  ltem_yellow_1;          // 黄色六边形
reg        [3:0]  ltem_yellow_2;          // 黄色圆形
reg        [3:0]  ltem_yellow_3;          // 黄色正方形  

reg        [31:0] target_tran;

parameter STATE_0  = 2'd0;                           
parameter STATE_1  = 2'd1;                

reg  [1:0]  state, next_state; 
reg         flag_0_to_1;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

delta_ctrl delta_ctrl_inst(
    .sys_clk(sys_clk),               
    .sys_rst_n(sys_rst_n),             
    .start_inst(start_inst),            
    .start_move(start_move),            
    .data_valid(data_valid),            
    .limit_switch_1(limit_switch_1),        
    .limit_switch_2(limit_switch_2),        
    .limit_switch_3(limit_switch_3),        
    .ltem_red_1(ltem_red_1),            
    .ltem_red_2(ltem_red_2),            
    .ltem_red_3(ltem_red_3),            
    .ltem_green_1(ltem_green_1),          
    .ltem_green_2(ltem_green_2),          
    .ltem_green_3(ltem_green_3),          
    .ltem_blue_1(ltem_blue_1),           
    .ltem_blue_2(ltem_blue_2),           
    .ltem_blue_3(ltem_blue_3),           
    .ltem_yellow_1(ltem_yellow_1),         
    .ltem_yellow_2(ltem_yellow_2),         
    .ltem_yellow_3(ltem_yellow_3),         
    .Delta_step_pulse_1(Delta_step_pulse_1),    
    .Delta_direction_1(Delta_direction_1),     
    .Delta_step_pulse_2(Delta_step_pulse_2),    
    .Delta_direction_2(Delta_direction_2),     
    .Delta_step_pulse_3(Delta_step_pulse_3),    
    .Delta_direction_3(Delta_direction_3),     
    .step_pulse_1(step_pulse_1),       
    .direction_1(direction_1),                 
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),    
    .electromagnet(electromagnet),         
    .sucker_pump(sucker_pump),           
    .sucker_valve(sucker_valve),  
    .tx(tx),
    .rx(rx)
);

tran_tape tran_tape_inst(
    .sys_clk(sys_clk),  
    .sys_rst_n(sys_rst_n),   
    .target_angles(target_tran),
    .step_pulse(step_pulse_tran),   
    .direction(direction_tran),    
    .reached_target(reached_target_tran)
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
            next_state = STATE_1;     // 否则保持在STATE_1
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end
        
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin  
        data_valid <= 0;
        target_tran <= 0;
        
        ltem_red_1    <= 0;
        ltem_red_2    <= 0;
        ltem_red_3    <= 0;
        ltem_green_1  <= 0;
        ltem_green_2  <= 0;
        ltem_green_3  <= 0;
        ltem_blue_1   <= 0;
        ltem_blue_2   <= 0;
        ltem_blue_3   <= 0;
        ltem_yellow_1 <= 0;
        ltem_yellow_2 <= 0;
        ltem_yellow_3 <= 0;
        
        flag_0_to_1   <= 1'b0;
    end
    else begin
        case(state)
            STATE_0: begin
                if(start_inst) begin

                    target_tran <= 13000;   

                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin  
                flag_0_to_1 <= 1'b0;             
                
                if(start_move) begin
                    data_valid <= 1;
                    target_tran <= 54272;
                    
                    ltem_red_1    <= 3;
                    ltem_red_2    <= 9;
                    ltem_red_3    <= 11;
                    ltem_green_1  <= 2;
                    ltem_green_2  <= 7;
                    ltem_green_3  <= 4;
                    ltem_blue_1   <= 8;
                    ltem_blue_2   <= 1;
                    ltem_blue_3   <= 5;
                    ltem_yellow_1 <= 12;
                    ltem_yellow_2 <= 10;
                    ltem_yellow_3 <= 6;   
                end
            end
            default: ; 
        endcase
    end
end 

endmodule
