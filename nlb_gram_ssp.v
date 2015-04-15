
`include "nlb_cfg_pkg.vh"

module nlb_gram_ssp (clk,      // input   clock
                we,        // input   write enable
                addr,     // input   write&read address with configurable width
                din,       // input   write data with configurable width
                dout);     // output  write data with configurable width

    parameter BUS_SIZE_ADDR =11;          // number of bits of address bus
    parameter BUS_SIZE_DATA =4;         // number of bits of data bus
    parameter GRAM_STYLE     =`GRAM_AUTO; // GRAM_AUTO, GRAM_BLCK, GRAM_DIST
    
    input                           clk;
    input                           we;
    input   [BUS_SIZE_ADDR-1:0]     addr;
    input   [BUS_SIZE_DATA-1:0]     din;
    output  [BUS_SIZE_DATA-1:0]     dout;
    
    (* `GRAM_STYLE = `GRAM_BLCK *)
    reg [BUS_SIZE_DATA-1:0] ram [(2**BUS_SIZE_ADDR)-1:0];
    
    reg [BUS_SIZE_DATA-1:0] dout;
    //reg [BUS_SIZE_DATA-1:0] ram_dout;
    
    always @(posedge clk)
    begin
      if (we) begin
        ram[addr]<=din; // synchronous write the RAM
      end
    end
//-----------------------------------------------------------------------
   always @(posedge clk)
   begin
      if (!we) begin
          dout    <= ram[addr];
      end
   end
    
endmodule
