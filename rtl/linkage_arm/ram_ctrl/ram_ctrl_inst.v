module ram_ctrl_inst
(
    input  wire                 sys_clk,             // 系统时钟
    input  wire                 sys_rst_n,           // 系统复位，低电平有效
       
    input  wire                 start_inst,          // 启动初始化机械臂
    input  wire                 start_move,          // 启动运动信号
       
    input  wire signed [15:0]   i_x,                 // 空间坐标X     
    input  wire signed [15:0]   i_y,                 // 空间坐标Y
    input  wire signed [15:0]   i_z,                 // 空间坐标Z
       
    input  wire                 turn_direction,      // 旋转轴方向
            
    output wire                 step_pulse_2,        // 旋转步进电机脉冲信号
    output wire                 direction_2,         // 旋转步进电机方向信号
    output wire                 reached_target_2,    // 旋转达到目标位置标志位    
            
    output wire                 step_pulse_3,        // 大臂步进电机脉冲信号
    output wire                 direction_3,         // 大臂步进电机方向信号 
    output wire                 reached_target_3,    // 大臂达到目标位置标志位
            
    output wire                 step_pulse_4,        // 小臂步进电机脉冲信号
    output wire                 direction_4,         // 小臂步进电机方向信号
    output wire                 reached_target_4,    // 小臂达到目标位置标志位
    
    output wire                 motion_done          //解算运动完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************// 

reg signed [15:0] target_x;
reg signed [15:0] target_y;
reg signed [15:0] target_z;

parameter STATE_0  = 3'b000;                           
parameter STATE_1  = 3'b001;                
parameter STATE_2  = 3'b010;                      
parameter STATE_3  = 3'b011;            
parameter STATE_4  = 3'b100;                           
parameter STATE_5  = 3'b101;                                     

reg        [2:0]  state, next_state; 
reg               flag_0_to_1;
reg               flag_1_to_2; 
reg               flag_2_to_3; 
reg               flag_3_to_4;
reg               flag_4_to_5;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

arm_motion_solver arm_motion_solver_inst(
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .i_x(target_x), 
    .i_y(target_y),             
    .i_z(target_z),             
    .turn_direction(turn_direction),  
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .step_pulse_4(step_pulse_4),
    .direction_4(direction_4),
    .reached_target_4(reached_target_4),
    .work_done(motion_done)       
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
            if(flag_3_to_4 == 1) 
                next_state = STATE_4; // 如果flag_3_to_4 == 1，转到STATE_4
            else 
                next_state = STATE_3; // 否则保持在STATE_3
        end
        STATE_4: begin
            if(flag_4_to_5 == 1) 
                next_state = STATE_5; // 如果flag_4_to_5 == 1，转到STATE_5
            else 
                next_state = STATE_4; // 否则保持在STATE_4
        end        
        STATE_5: begin 
                next_state = STATE_5; // 保持在STATE_5
        end      
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end        
        
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin  
        target_x <= 16'd0;
        target_y <= 16'd0;
        target_z <= 16'd0;
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0; 
        flag_2_to_3 <= 0; 
        flag_3_to_4 <= 0;
        flag_4_to_5 <= 0;
    end
    else begin
        case(state)
            STATE_0: begin
                if(start_inst) begin
                    target_x <= 16'd190;
                    target_y <= 16'd0;
                    target_z <= 16'd275;
                
                    flag_0_to_1 <= 1;
                end
                else begin
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin  
                flag_0_to_1 <= 0;  
                
                if (!motion_done) begin

                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
            end            
            STATE_2: begin
                flag_1_to_2 <= 0;
                      
                if (motion_done && start_inst) begin
                    target_x <= 16'd250;
                    target_y <= 16'd0;
                    target_z <= 16'd150; 
                    
                    flag_2_to_3 <= 1;
                end            
                else if (!motion_done) begin
                    flag_2_to_3 <= 0;                    
                end
            end 
            STATE_3: begin  
                flag_2_to_3 <= 0;  
                
                if (!motion_done) begin

                    flag_3_to_4 <= 1;
                end
                else begin
                    flag_3_to_4 <= 0;
                end
            end 
            STATE_4: begin  
                flag_3_to_4 <= 0;  
                
                if(start_move && motion_done) begin
                
                    flag_4_to_5 <= 1;
                end               
                else begin
                    flag_4_to_5 <= 0;
                end
            end 
            STATE_5: begin  
                flag_4_to_5 <= 0;
            
                target_x <= i_x;
                target_y <= i_y;
                target_z <= i_z;
            end                
            default: ; 
        endcase
    end
end 

endmodule
