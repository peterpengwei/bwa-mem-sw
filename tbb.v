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

module tbb #(parameter TBB_WR_ADDR_WIDTH=12, 
                       TBB_WR_DATA_WIDTH=512, 
                       TBB_RD_ADDR_WIDTH=16, 
                       TBB_RD_DATA_WIDTH=32)
(
    // ---------------------------global signals-------------------------------------------------
    clk,                              //              in    std_logic;  -- Core clock
    reset_n,                          //              in    std_logic;  -- Use SPARINGLY only for control
    // ---------------------------buffer r/w signals---------------------------------------------
    task_start,
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

    input                                clk;                  //              in    std_logic;  -- Core clock
    input                                reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control
    input                                task_done;
    input                                ReqAck;
    input                                WrEn;
    input   [TBB_RD_ADDR_WIDTH-1:0]      RdAddr;
    input   [TBB_WR_ADDR_WIDTH-1:0]      WrAddr;
    input   [TBB_WR_DATA_WIDTH-1:0]      WrDin;

    output                               Full;
    output                               Empty;
    output                               ReqValid;
    output                               task_start;
    output  [TBB_WR_ADDR_WIDTH-1:0]      ReqLineIdx;
    output  [TBB_RD_DATA_WIDTH-1:0]      RdDout;

    reg [4:0]                            cur_state;
    reg [4:0]                            next_state;

    localparam RESET         = 'b00001;
    localparam READ          = 'b00010;
    localparam WAIT          = 'b00100;
    localparam WRITE         = 'b01000;
    localparam HOLD          = 'b10000;
    localparam NUM_LINES     = 1 << TBB_WR_ADDR_WIDTH;

    reg [TBB_WR_ADDR_WIDTH-1:0]	    wr_counter_d;
    reg [TBB_WR_ADDR_WIDTH-1:0]	    wr_counter;
    reg [TBB_WR_ADDR_WIDTH-1:0]	    req_counter_d;
    reg [TBB_WR_ADDR_WIDTH-1:0]	    req_counter;

    //-------------------------
    // STATE_MACHINE
    //-------------------------
    always @(posedge clk)
    begin
        if(!reset_n)
        begin
           cur_state <= RESET;
        end
        else
        begin                  
           cur_state <= next_state;
        end
    end

    //-------------------------
    // Next State
    //-------------------------
    always @ (*)
    begin
        next_state = RESET;
        case(cur_state)         /* synthesis parallel_case */
            RESET:        
            begin
                next_state = READ;
            end
            READ:
            begin
                if (ReqAck && req_counter == (NUM_LINES-1))
                    next_state = WAIT;
                else
                    next_state = READ;
            end	
            WAIT:
            begin
                next_state = WRITE;
            end
            WRITE:
            begin
                if (WrEn && wr_counter == (NUM_LINES-1))
                    next_state = HOLD;
                else
                    next_state = WRITE;
            end	
            HOLD:
            begin
                if (task_done)
                    next_state = READ;
                else
                    next_state = HOLD;
            end
        endcase
    end

    //-------------------------
    // Output
    //-------------------------
    assign task_start       = (cur_state == WRITE) && (next_state == HOLD);
    assign ReqValid         = (cur_state == READ);
    assign ReqLineIdx       = req_counter;
    assign Full             = (cur_state == RESET || cur_state == WRITE || cur_state == HOLD);
    assign Empty            = ~Full;
    
    always @ (posedge clk)
    begin
        if (!reset_n)
        begin
            wr_counter <= 'b0;
            req_counter <= 'b0;
        end
        else
        begin
            wr_counter <= wr_counter_d;
            req_counter <= req_counter_d;
        end
    end
    
    always @ (*)
    begin
        wr_counter_d = wr_counter;
        req_counter_d = req_counter;
        if (WrEn) 
        begin
            if (wr_counter == (NUM_LINES-1))
                wr_counter_d = 'b0;
            else
                wr_counter_d = wr_counter + 'b1;
        end
        if (ReqAck) 
        begin
            if (req_counter == (NUM_LINES-1))
                req_counter_d = 'b0;
            else
                req_counter_d = req_counter + 'b1;
        end
    end

    wire    [TBB_WR_ADDR_WIDTH-1:0] RdAddr_BRAM = RdAddr[TBB_RD_ADDR_WIDTH-1:4];
    wire    [TBB_WR_DATA_WIDTH-1:0] RdDout_BRAM;
    reg     [TBB_RD_DATA_WIDTH-1:0] RdDout;
    reg     [3:0] RdAddr_Low_r;
    
    always @(posedge clk)
    begin
        RdAddr_Low_r <= RdAddr[3:0];
    end

    always @(*)
    begin
        RdDout = 'b0;
        case(RdAddr_Low_r)         /* synthesis parallel_case */
        'd0:	RdDout = RdDout_BRAM[31+0*32:0*32];
        'd1:	RdDout = RdDout_BRAM[31+1*32:1*32];
        'd2:	RdDout = RdDout_BRAM[31+2*32:2*32];
        'd3:	RdDout = RdDout_BRAM[31+3*32:3*32];
        'd4:	RdDout = RdDout_BRAM[31+4*32:4*32];
        'd5:	RdDout = RdDout_BRAM[31+5*32:5*32];
        'd6:	RdDout = RdDout_BRAM[31+6*32:6*32];
        'd7:	RdDout = RdDout_BRAM[31+7*32:7*32];
        'd8:	RdDout = RdDout_BRAM[31+8*32:8*32];
        'd9:	RdDout = RdDout_BRAM[31+9*32:9*32];
        'd10:	RdDout = RdDout_BRAM[31+10*32:10*32];
        'd11:	RdDout = RdDout_BRAM[31+11*32:11*32];
        'd12:	RdDout = RdDout_BRAM[31+12*32:12*32];
        'd13:	RdDout = RdDout_BRAM[31+13*32:13*32];
        'd14:	RdDout = RdDout_BRAM[31+14*32:14*32];
        'd15:	RdDout = RdDout_BRAM[31+15*32:15*32];
        endcase
    end

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------
    nlb_gram_sdp #(.BUS_SIZE_ADDR(TBB_WR_ADDR_WIDTH),
                   .BUS_SIZE_DATA(TBB_WR_DATA_WIDTH),
                   .GRAM_MODE(2'd1)
    )tbb_bram 
    (
        .clk  (clk),
        .we   (WrEn),
        .waddr(WrAddr),
        .din  (WrDin),
        .raddr(RdAddr_BRAM),
        .dout (RdDout_BRAM)
    );

endmodule
