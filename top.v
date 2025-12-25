//`include "cpu.v"
//`include "progmem.v"
//`include "uart_gpio.v"
module top(
    input rst, clk,
    output uart_tx
  );
  wire [31:0] mem_rdata, mem_wdata, addr;
  wire rstrb;
  wire [3:0] wr_strobe;
  wire [31:0] uart_status;
//select device
  wire isMEM = (addr[31:28]==4'b0000);
  //wire isLED = (addr[31:28]==4'b0001);
  //wire isUART_DATA= (addr[31:28]==4'b0010);
  //wire isUART_CTRL= (addr[31:28]==4'b0011);
  wire isUART_STATUS= (addr[31:28]==4'b0100);
  wire [31:0] cpu_rdata = isUART_STATUS ? uart_status: 
                         isMEM ? mem_rdata: 32'h0;

  //Instantiate sub modules
  cpu cpu0(
        .rst(!rst), .clk(clk),
        .mem_rdata(cpu_rdata),
        .mem_addr(addr),
        .cycle(),
        .mem_rstrb(rstrb),
        .mem_wdata(mem_wdata),
        .mem_wstrb(wr_strobe)
      );

  progmem mem0(
            .rst(!rst), .clk(clk),
            .addr(addr),
            .data_in(mem_wdata),
            .rd_strobe(rstrb & isMEM),
            .wr_strobe(wr_strobe & {4{isMEM}}),
            .data_out(mem_rdata)
          );

  uart_gpio uart0(
                .rst(!rst), .clk(clk),
                .addr(addr),
                .data_in(mem_wdata),
                .rd_strobe(rstrb ),
                .wr_strobe(wr_strobe),
                .data_out(uart_status),
                .tx_pin(uart_tx)
                );

  
endmodule
