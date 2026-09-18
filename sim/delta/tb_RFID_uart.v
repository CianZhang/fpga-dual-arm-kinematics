`timescale 1ns / 1ps

module tb_RFID_uart;

    // 参数定义
    parameter CLK_PERIOD = 20; // 50MHz 时钟周期，20ns
    parameter UART_BPS = 115200; // 波特率
    parameter CLK_FREQ = 50_000_000; // 时钟频率
    parameter BIT_TIME = 1_000_000_000 / UART_BPS; // 每个比特的持续时间（ns）

    // 测试信号
    reg         sys_clk;
    reg         sys_rst_n;
    reg         send_trig;
    reg         rx;
    wire        tx;
    wire [2:0]  match_id;
    wire        data_valid;

    // 实例化 DUT
    RFID_uart #(
        .UART_BPS(UART_BPS),
        .CLK_FREQ(CLK_FREQ)
    ) uut (
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .send_trig(send_trig),
        .rx(rx),
        .tx(tx),
        .match_id(match_id),
        .data_valid(data_valid)
    );

    // 时钟生成
    initial begin
        sys_clk = 0;
        forever #(CLK_PERIOD / 2) sys_clk = ~sys_clk;
    end

    // 发送 UART 数据函数
    task send_uart_byte;
        input [7:0] data;
        integer i;
        begin
            // 起始位
            rx = 0;
            #BIT_TIME;
            // 数据位
            for (i = 0; i < 8; i = i + 1) begin
                rx = data[i];
                #BIT_TIME;
            end
            // 停止位
            rx = 1;
            #BIT_TIME;
        end
    endtask

    // 发送数据包任务（直接指定数据包内容）
    task send_packet;
        input [7:0] byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11;
        begin
            send_uart_byte(byte0);
            send_uart_byte(byte1);
            send_uart_byte(byte2);
            send_uart_byte(byte3);
            send_uart_byte(byte4);
            send_uart_byte(byte5);
            send_uart_byte(byte6);
            send_uart_byte(byte7);
            send_uart_byte(byte8);
            send_uart_byte(byte9);
            send_uart_byte(byte10);
            send_uart_byte(byte11);
        end
    endtask

    // 初始化和测试用例
    initial begin
        // 初始化信号
        sys_rst_n = 0;
        send_trig = 0;
        rx = 1; // UART 空闲状态为高电平
        #100;
        sys_rst_n = 1;
        #100;

        // 测试用例 1：发送触发，接收有效数据包 (last byte = ED)
        $display("Test Case 1: Send trigger and receive valid packet with last byte ED");
        send_trig = 1;
        #CLK_PERIOD;
        send_trig = 0;
        #100000; // 等待发送完成（粗略估计）

        // 发送有效数据包：02 0C B0 30 00 04 00 00 00 00 00 ED
        send_packet(8'h02, 8'h0C, 8'hB0, 8'h30, 8'h00, 8'h04, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'hED);
        #10000; // 等待处理
        if (match_id == 3'd1 && data_valid == 1)
            $display("Test Case 1 PASSED: match_id = %d, data_valid = %b", match_id, data_valid);
        else
            $display("Test Case 1 FAILED: match_id = %d, data_valid = %b", match_id, data_valid);

        // 测试用例 2：发送触发，接收无效数据包 (header = 04 0C 02)
        $display("Test Case 2: Send trigger and receive invalid packet");
        send_trig = 1;
        #CLK_PERIOD;
        send_trig = 0;
        #100000;

        // 发送无效数据包：04 0C 02 30 00 04 00 00 00 00 00 ED
        send_packet(8'h04, 8'h0C, 8'h02, 8'h30, 8'h00, 8'h04, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'hED);
        #10000;
        if (match_id == 3'd0 && data_valid == 0)
            $display("Test Case 2 PASSED: match_id = %d, data_valid = %b", match_id, data_valid);
        else
            $display("Test Case 2 FAILED: match_id = %d, data_valid = %b", match_id, data_valid);

        // 测试用例 3：发送触发，接收有效数据包 (last byte = C5)
        $display("Test Case 3: Send trigger and receive valid packet with last byte C5");
        send_trig = 1;
        #CLK_PERIOD;
        send_trig = 0;
        #100000;

        // 发送有效数据包：02 0C B0 30 00 04 00 00 00 00 00 C5
        send_packet(8'h02, 8'h0C, 8'hB0, 8'h30, 8'h00, 8'h04, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'hC5);
        #10000;
        if (match_id == 3'd2 && data_valid == 1)
            $display("Test Case 3 PASSED: match_id = %d, data_valid = %b", match_id, data_valid);
        else
            $display("Test Case 3 FAILED: match_id = %d, data_valid = %b", match_id, data_valid);

        // 测试用例 4：发送触发，接收有效数据包 (last byte = 87)
        $display("Test Case 4: Send trigger and receive valid packet with last byte 87");
        send_trig = 1;
        #CLK_PERIOD;
        send_trig = 0;
        #100000;

        // 发送有效数据包：02 0C B0 30 00 04 00 00 00 00 00 87
        send_packet(8'h02, 8'h0C, 8'hB0, 8'h30, 8'h00, 8'h04, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h87);
        #10000;
        if (match_id == 3'd3 && data_valid == 1)
            $display("Test Case 4 PASSED: match_id = %d, data_valid = %b", match_id, data_valid);
        else
            $display("Test Case 4 FAILED: match_id = %d, data_valid = %b", match_id, data_valid);

        // 测试用例 5：发送触发，接收有效数据包 (last byte = 1A)
        $display("Test Case 5: Send trigger and receive valid packet with last byte 1A");
        send_trig = 1;
        #CLK_PERIOD;
        send_trig = 0;
        #100000;

        // 发送有效数据包：02 0C B0 30 00 04 00 00 00 00 00 1A
        send_packet(8'h02, 8'h0C, 8'hB0, 8'h30, 8'h00, 8'h04, 8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h1A);
        #10000;
        if (match_id == 3'd4 && data_valid == 1)
            $display("Test Case 5 PASSED: match_id = %d, data_valid = %b", match_id, data_valid);
        else
            $display("Test Case 5 FAILED: match_id = %d, data_valid = %b", match_id, data_valid);

        // 结束仿真
        #10000;
        $display("Simulation finished");
        $stop;
    end

    // 监控输出
    initial begin
        $monitor("Time=%0t, match_id=%d, data_valid=%b, tx=%b, rx=%b", 
                 $time, match_id, data_valid, tx, rx);
    end

endmodule