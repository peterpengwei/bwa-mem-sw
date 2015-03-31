// ***************************************************************************
//
//        Task Batch Buffer
//
// Engineer:            Peng Wei
// Create Date:         Mar 18, 2015
// Module Name:         tbb
// Description:         A task batch buffer contains a task batch for a specific PE Array to process
//                      it instantiates tbb_bram
// ***************************************************************************

module tbb #(parameter TBB_ADDR_WIDTH, TBB_DATA_WIDTH)
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
   input [TBB_ADDR_WIDTH-1:0]		RdAddr;
   output [TBB_DATA_WIDTH-1:0]		RdDout;
   output 				Empty;

   reg [2:0]				cur_state;
   reg [2:0]				next_state;

   localparam IDLE			'b001
   localparam READ			'b010
   localparam HOLD			'b100
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
			next_state = READ;
		end
                READ:
		begin
			if (WrEn && wr_counter == NUM_LINES-1)
				next_state = HOLD;
			else
				next_state = READ;
		end	
                HOLD:
		begin
			if (task_done)
				next_state = IDLE;
			else
				next_state = HOLD;
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
    wire Full = (cur_state == HOLD);
    wire Empty = ~Full;
    wire request = (cur_state == READ);
    reg [NUM_LINES-1:0]		wr_counter_d;
    reg [NUM_LINES-1:0]		wr_counter;
    always @ (posedge clk)
    begin
	    if (!reset_n)
		    wr_counter <= 0;
	    else
		    wr_counter <= wr_counter_d + 1;
    end
    always @ (*)
    begin
	wr_counter_d = wr_counter;
	if (cur_state == READ && WrEn) 
	begin
		if (wr_counter == NUM_LINES-1)
			wr_counter_d = 0;
		else
			wr_counter_d = wr_counter + 1;
	end
    end

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------
   nlb_gram_sdp #(.BUS_SIZE_ADDR(TBB_ADDR_WIDTH),
              .BUS_SIZE_DATA(TBB_DATA_WIDTH),
              .GRAM_MODE(2'd1)
              )tbb_bram 
            (
                .clk  (clk),
                .we   (WrEn),        
                .waddr(WrAddr),     
                .din  (WrDin),       
                .raddr(RdAddr),     
                .dout (RdDout)
            );     

endmodule
