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
    task_done,
    ReqValid,
    ReqLineIdx,
    ReqAck,
    WrEn, 	         			
    WrAddr,					
    WrDin,					
    Full, 					
    RdAddr,					
    RdDout,					
    Empty 					
);

   input                        clk;                  //              in    std_logic;  -- Core clock
   input                        reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control

   input				task_done;
   output				ReqValid;
   output [RBB_ADDR_WIDTH-1:0]		ReqLineIdx;
   input 				ReqAck;
   input 				WrEn;
   input [RBB_ADDR_WIDTH-1:0]		WrAddr;
   input [RBB_DATA_WIDTH-1:0]		WrDin;
   output 				Full;

   input [RBB_ADDR_WIDTH-1:0]		RdAddr;
   output [RBB_DATA_WIDTH-1:0]		RdDout;
   output 				Empty;

   reg [1:0]				cur_state;
   reg [1:0]				next_state;

   localparam IDLE			'b01
   localparam WRITE			'b10
   localparam NUM_LINES			(1 << RBB_ADDR_WIDTH)

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
			if (ReqAck && rd_counter == 'd(NUM_LINES-1))
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
    reg [RBB_ADDR_WIDTH-1:0]		rd_counter_d;
    reg [RBB_ADDR_WIDTH-1:0]		rd_counter;
    wire Full = (cur_state == WRITE);
    wire Empty = ~Full;
    wire [RBB_ADDR_WIDTH-1:0] RdAddrBRAM = (cur_state == IDLE && next_state == WRITE) ? 'b0 : rd_counter + 'b1;
    reg [RBB_ADDR_WIDTH-1:0] RdAddrBRAM_r;
    always @ (posedge clk)
    begin
	    RdAddrBRAM_r <= RdAddrBRAM;
    end
    wire [RBB_DATA_WIDTH-1:0] RdDataBRAM;
    wire ReqValid = (cur_state == WRITE);

    reg [RBB_ADDR_WIDTH-1:0] ReqLineIdx;
    reg [RBB_ADDR_WIDTH-1:0] ReqLineIdx_r;
    reg [RBB_DATA_WIDTH-1:0] RdDout;
    reg [RBB_DATA_WIDTH-1:0] RdDout_r;
    reg task_done_r;
    reg ReqAck_r;
    always @ (posedge clk)
    begin
	    ReqLineIdx_r <= ReqLineIdx;
	    RdDout_r <= RdDout;
	    task_done_r <= task_done;
	    ReqAck_r <= ReqAck;
    end
    always @ (*)
    begin
	    if (task_done_r || ReqAck_r)
	    begin
		    ReqLineIdx = RdAddrBRAM_r;
		    RdDout = RdDataBRAM;
	    end
	    else
	    begin
		    ReqLineIdx = ReqLineIdx_r;
		    RdDout = RdDout_r;
	    end
    end

    always @ (posedge clk)
    begin
	    if (!reset_n)
		    rd_counter <= 0;
	    else
		    rd_counter <= rd_counter_d;
    end
    always @ (*)
    begin
	rd_counter_d = rd_counter;
	if (cur_state == WRITE && ReqAck) 
	begin
		if (rd_counter == 'd(NUM_LINES-1))
			rd_counter_d = 'b0;
		else
			rd_counter_d = rd_counter + 'b1;
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
                .raddr(RdAddrBRAM),     
                .dout (RdDataBRAM)
            );     

endmodule
