`timescale 1ns / 1ps

module tb_uart_rx_top;

    // 参数定义
    parameter CLK_FREQ    = 50_000_000;     // 时钟频率 50MHz
    parameter UART_BPS    = 9600;           // 波特率 9600
    parameter CLK_PERIOD  = 20;             // 时钟周期 20ns (50MHz)
    parameter BAUD_CYCLE = CLK_FREQ / UART_BPS * CLK_PERIOD; // 每个比特的时长 (ns)

    // 信号定义
    reg         sys_clk;
    reg         sys_rst_n;
    reg         rx;
    wire [15:0] rx_data1;
    wire [15:0] rx_data2;
    wire [15:0] rx_data3;
    wire        rx_done;

    // 实例化顶层模块
    uart_rx_top #(
        .UART_BPS(UART_BPS),
        .CLK_FREQ(CLK_FREQ)
    ) uut (
        .sys_clk(sys_clk),
        .sys_rst_n(sys_rst_n),
        .rx(rx),
        .rx_data1(rx_data1),
        .rx_data2(rx_data2),
        .rx_data3(rx_data3),
        .rx_done(rx_done)
    );

    // 时钟生成
    initial begin
        sys_clk = 0;
        forever #(CLK_PERIOD/2) sys_clk = ~sys_clk;
    end

    // 任务：发送一个 UART 字节
    task uart_send_byte;
        input [7:0] data;
        integer j;
        begin
            // 起始位 (0)
            rx = 1'b0;
            #(BAUD_CYCLE);
            // 8 位数据 (LSB 先传)
            for (j = 0; j < 8; j = j + 1) begin
                rx = data[j];
                #(BAUD_CYCLE);
            end
            // 停止位 (1)
            rx = 1'b1;
            #(BAUD_CYCLE);
            // 额外等待，确保字节间隙
            #(BAUD_CYCLE * 2);
        end
    endtask

    // 测试流程
    initial begin
        // 初始化
        sys_rst_n = 0;
        rx = 1;
        #100;
        sys_rst_n = 1;
        #100;

        // 测试用例 1：接收数据包
        $display("=== Test Case 1: Receive Packet ===");
        // 数据包内容：包头 + {+32767, -32768, -1} + 包尾
        $display("Sending byte 0: 0xAA");
        uart_send_byte(8'hAA);              // 包头
        $display("Sending byte 1: 0x00");
        uart_send_byte(8'h00);              // data1 低字节 (0x7FFF)
        $display("Sending byte 2: 0x00");
        uart_send_byte(8'h00);              // data1 高字节
        $display("Sending byte 3: 0x00");
        uart_send_byte(8'h00);              // data2 低字节 (0x8000)
        $display("Sending byte 4: 0x00");
        uart_send_byte(8'h00);              // data2 高字节
        $display("Sending byte 5: 0x40");
        uart_send_byte(8'h40);              // data3 低字节 (0xFFFF)
        $display("Sending byte 6: 0xED");
        uart_send_byte(8'hED);              // data3 高字节
        $display("Sending byte 7: 0x55");
        uart_send_byte(8'h55);              // 包尾

        // 测试用例 2：接收错误包头
        #1000;
        $display("\n=== Test Case 2: Receive Packet with Wrong Header ===");
        $display("Sending byte 0: 0xAB");
        uart_send_byte(8'hAA); 
        $display("Sending byte 1: 0xFF");
        uart_send_byte(8'hFF);
        $display("Sending byte 2: 0x7F");
        uart_send_byte(8'h7F);
        $display("Sending byte 3: 0x00");
        uart_send_byte(8'h00);
        $display("Sending byte 4: 0x80");
        uart_send_byte(8'h80);
        $display("Sending byte 5: 0xFF");
        uart_send_byte(8'hFF);
        $display("Sending byte 6: 0xFF");
        uart_send_byte(8'hFF);
        $display("Sending byte 7: 0x55");
        uart_send_byte(8'h55);
        #1000;

        // 结束仿真
        #1000;
        $display("\n=== Simulation Finished ===");
        $stop;
    end

endmodule