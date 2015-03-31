// ***************************************************************************
//
//        Result Batch Buffer
//
// Engineer:            Peng Wei
// Create Date:         Mar 20, 2015
// Module Name:         rbb
// Description:         A result batch buffer contains a result batch from a specific PE Array to process
//                      it instantiates rbb_bram
// ***************************************************************************

module rbb #(parameter RBB_ADDR_WIDTH, RBB_DATA_WIDTH)
(
    // ---------------------------global signals-------------------------------------------------
    clk,                              //              in    std_logic;  -- Core clock
    reset_n,                          //              in    std_logic;  -- Use SPARINGLY only for control
    // ---------------------------buffer r/w signals---------------------------------------------
    request,
    task_done,
    WrEn, 	         			
    WrAddr,					
    WrDin,					
    Full, 					
    RdEn, 					
    RdAddr,					
    RdDout,					
    Empty 					
);

   input                        clk;                  //              in    std_logic;  -- Core clock
   input                        reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control

   output				request;
   input				task_done;
   input 				WrEn;
   input [RBB_ADDR_WIDTH-1:0]		WrAddr;
   input [RBB_DATA_WIDTH-1:0]		WrDin;
   output 				Full;

   input 				RdEn;
   input [RBB_ADDR_WIDTH-1:0]		RdAddr;
   output [RBB_DATA_WIDTH-1:0]		RdDout;
   output 				Empty;

   reg [1:0]				cur_state;
   reg [1:0]				next_state;

   localparam IDLE			'b01
   localparam WRITE			'b10
   localparam NUM_LINES			'd16

    //-------------------------
    //STATE_MACHINE
    //-------------------------
    always @(posedge clk)                                              
    begin                                                                   
        if(!reset_n)
        begin
	    cur_state <= IDLE;
        end
        else
        begin                  
	    cur_state <= next_state;
        end
    end

    //-------------------------
    //Next State
    //-------------------------
    always @ (*)
    begin
	    next_state = IDLE;
            case(cur_state)         /* synthesis parallel_case */
                IDLE:        
		begin
			if (task_done)
				next_state = WRITE;
			else
				next_state = IDLE;
		end
                WRITE:
		begin
			if (RdEn && rd_counter == NUM_LINES-1)
				next_state = IDLE;
			else
				next_state = WRITE;
		end	
		default:
		begin
			next_state = IDLE;
		end	
            endcase
    end

    //-------------------------
    //Output
    //-------------------------
    wire Full = (cur_state == WRITE);
    wire Empty = ~Full;
    wire request = (cur_state == WRITE);
    reg [NUM_LINES-1:0]		rd_counter_d;
    reg [NUM_LINES-1:0]		rd_counter;
    always @ (posedge clk)
    begin
	    if (!reset_n)
		    rd_counter <= 0;
	    else
		    rd_counter <= rd_counter_d + 1;
    end
    always @ (*)
    begin
	rd_counter_d = rd_counter;
	if (cur_state == WRITE && RdEn) 
	begin
		if (rd_counter == NUM_LINES-1)
			rd_counter_d = 0;
		else
			rd_counter_d = rd_counter + 1;
	end
    end

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------
   nlb_gram_sdp #(.BUS_SIZE_ADDR(RBB_ADDR_WIDTH),
              .BUS_SIZE_DATA(RBB_DATA_WIDTH),
              .GRAM_MODE(2'd1)
              )rbb_bram 
            (
                .clk  (clk),
                .we   (WrEn),        
                .waddr(WrAddr),     
                .din  (WrDin),       
                .raddr(RdAddr),     
                .dout (RdDout)
            );     

endmodule
