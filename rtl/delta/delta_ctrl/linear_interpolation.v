module linear_interpolation
(
    input wire                  sys_clk,             // 系统时钟，50MHz
    input wire                  sys_rst_n,           // 系统复位，低电平有效
    input wire                  start,               // 启动信号
    input wire signed [15:0]    x_16,                // 目标空间坐标 X
    input wire signed [15:0]    y_16,                // 目标空间坐标 Y
    input wire signed [15:0]    z_16,                // 目标空间坐标 Z
    
    output wire                 step_pulse_1,        // 电机 1 脉冲信号
    output wire                 direction_1,         // 电机 1 方向信号
    output wire                 reached_target_1,    // 电机 1 到达标志
    output wire                 step_pulse_2,        // 电机 2 脉冲信号
    output wire                 direction_2,         // 电机 2 方向信号
    output wire                 reached_target_2,    // 电机 2 到达标志
    output wire                 step_pulse_3,        // 电机 3 脉冲信号
    output wire                 direction_3,         // 电机 3 方向信号
    output wire                 reached_target_3,    // 电机 3 到达标志
    
    output reg                  move_done            // 插补完成信号
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//

parameter INTERP_STEPS = 64;            // 插补步数，控制平滑度
parameter INV_INTERP_STEPS = 1024;        // 1/128 * 2^16 ≈  * 65536，用于定点数计算

reg signed [15:0]   start_x;
reg signed [15:0]   start_y;
reg signed [15:0]   start_z;

reg signed [15:0]   end_x;
reg signed [15:0]   end_y;
reg signed [15:0]   end_z;

reg signed [15:0]   current_x; 
reg signed [15:0]   current_y; 
reg signed [15:0]   current_z; 

reg signed [31:0]   delta_x;
reg signed [31:0]   delta_y;
reg signed [31:0]   delta_z;

reg                 delta_x_flag;
reg                 delta_y_flag;
reg                 delta_z_flag;

reg        [15:0]   current_step;       // 当前插补步数
reg        [15:0]   cnt;

wire                motion_done;        // 运动解算完成信号

parameter STATE_0 = 2'b00;                           
parameter STATE_1 = 2'b01;                                                                           
parameter STATE_2 = 2'b10;                                                                           
parameter STATE_3 = 2'b11;

reg          [1:0]  state, next_state; 
reg                 flag_0_to_1;
reg                 flag_1_to_2;
reg                 flag_2_to_1;
reg                 flag_2_to_3;
reg                 flag_3_to_0;

//********************************************************************//
//************************** Submodule Inst **************************//
//********************************************************************//

motion_solver motion_solver_inst (
    .sys_clk(sys_clk),
    .sys_rst_n(sys_rst_n),
    .x_16(current_x),
    .y_16(current_y),
    .z_16(current_z),
    .step_pulse_1(step_pulse_1),
    .direction_1(direction_1),
    .reached_target_1(reached_target_1),
    .step_pulse_2(step_pulse_2),
    .direction_2(direction_2),
    .reached_target_2(reached_target_2),
    .step_pulse_3(step_pulse_3),
    .direction_3(direction_3),
    .reached_target_3(reached_target_3),
    .motion_solver_work_done(motion_done)
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
            if (flag_2_to_1 == 1 && flag_2_to_3 == 0)
                next_state = STATE_1;
            else if (flag_2_to_3 == 1)
                next_state = STATE_3;
            else 
                next_state = STATE_2;
        end
        STATE_3: begin
            if(flag_3_to_0 == 1)
                next_state = STATE_0; // 如果flag_3_to_0 == 1，转到STATE_0
            else 
                next_state = STATE_3; // 否则保持在STATE_3
        end
        default: next_state = STATE_0; // 默认转到初始状态
    endcase
end 

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_x <= 16'sd0;
        start_y <= 16'sd0;
        start_z <= -16'sd3800;
        current_x <= 16'sd0;
        current_y <= 16'sd0;
        current_z <= -16'sd3800;
        end_x <= 16'sd0;
        end_y <= 16'sd0;
        end_z <= -16'sd3800;
        delta_x <= 0;
        delta_y <= 0; 
        delta_z <= 0;
        current_step <= 0;
        move_done <= 1;
        cnt <= 0;
        
        delta_x_flag <= 0;
        delta_y_flag <= 0;
        delta_z_flag <= 0;  
        
        flag_0_to_1 <= 0;
        flag_1_to_2 <= 0;
        flag_2_to_1 <= 0;
        flag_2_to_3 <= 0;
        flag_3_to_0 <= 0;        
    end
    else begin
        case (state)
            STATE_0: begin
                move_done <= 1;
                flag_3_to_0 <= 0;

                if(z_16 !=0) begin
                    if (start && (x_16 != start_x || y_16 != start_y || z_16 != start_z)) begin
                        move_done <= 0;
                        current_step <= 1;
                        
                        if(((x_16 - start_x) > INTERP_STEPS) || ((start_x - x_16) > INTERP_STEPS)) begin
                            delta_x <= (x_16 - start_x);
                            delta_x_flag <= 1;
                        end
                        else begin
                            delta_x_flag <= 0;
                        end
                        if(((y_16 - start_y) > INTERP_STEPS) || ((start_y - y_16) > INTERP_STEPS)) begin
                            delta_y <= (y_16 - start_y);
                            delta_y_flag <= 1;
                        end
                        else begin
                            delta_y_flag <= 0;
                        end                            
                        if(((z_16 - start_z) > INTERP_STEPS) || ((start_z - z_16) > INTERP_STEPS)) begin
                            delta_z <= (z_16 - start_z);
                            delta_z_flag <= 1;
                        end
                        else begin
                            delta_z_flag <= 0;
                        end 
                        
                        end_x <= x_16;
                        end_y <= y_16;
                        end_z <= z_16;
                        
                        flag_0_to_1 <= 1;
                    end
                    else begin
                        flag_0_to_1 <= 0;
                    end
                end
                else begin 
                    flag_0_to_1 <= 0;
                end
            end
            STATE_1: begin
                flag_0_to_1 <= 0;
                flag_2_to_1 <= 0;
                
                if(delta_x_flag) begin
                    current_x <= start_x + ((delta_x * current_step * INV_INTERP_STEPS) >> 16);
                end
                if(delta_y_flag) begin
                    current_y <= start_y + ((delta_y * current_step * INV_INTERP_STEPS) >> 16);
                end
                if(delta_z_flag) begin
                    current_z <= start_z + ((delta_z * current_step * INV_INTERP_STEPS) >> 16);
                end
                
                cnt <= cnt + 1; 
                if(cnt > 10) begin
                    cnt <= 0; 
                    flag_1_to_2 <= 1;
                end
                else begin
                    flag_1_to_2 <= 0;
                end
                
                flag_1_to_2 <= 1;
            end
            STATE_2: begin
                flag_1_to_2 <= 0;
            
                if(cnt <= 50) begin 
                    cnt <= cnt + 1;
                end
                else begin
                  if (motion_done) begin
                      cnt <= 0;
                  
                        current_step <= current_step + 1;
                        if (current_step >= (INTERP_STEPS - 1)) begin
                            start_x <= end_x;
                            start_y <= end_y;
                            start_z <= end_z;
                            current_x <= end_x;
                            current_y <= end_y;
                            current_z <= end_z;
                            flag_2_to_1 <= 0;
                            flag_2_to_3 <= 1;
                        end
                        else begin
                            flag_2_to_1 <= 1;
                            flag_2_to_3 <= 0;
                        end
                    end
                end
            end
            STATE_3: begin
                flag_2_to_3 <= 0;
                cnt <= 0;
                
                delta_x_flag <= 0;
                delta_y_flag <= 0;
                delta_z_flag <= 0;
                
                if (motion_done) begin
                    move_done <= 1;
                    flag_3_to_0 <= 1;
                end
                else begin
                    flag_3_to_0 <= 0;
                end
            end
            default: ; 
        endcase
    end
end

endmodule
