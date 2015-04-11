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

module rbb #(parameter RBB_RD_ADDR_WIDTH=8, RBB_RD_DATA_WIDTH=512, RBB_WR_ADDR_WIDTH=12, RBB_WR_DATA_WIDTH=32)
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
    Empty,
    TestCmp
);

    input                               clk;                  //              in    std_logic;  -- Core clock
    input                               reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control
    input                               task_done;
    input                               ReqAck;
    input                               WrEn;
    input   [RBB_WR_ADDR_WIDTH-1:0]     WrAddr;
    input   [RBB_WR_DATA_WIDTH-1:0]     WrDin;
    input   [RBB_RD_ADDR_WIDTH-1:0]     RdAddr;
    
    output                              Full;
    output                              Empty;
    output  [RBB_RD_ADDR_WIDTH-1:0]     ReqLineIdx;
    output                              ReqValid;
    output  [RBB_RD_DATA_WIDTH-1:0]     RdDout;
    output                              TestCmp;

    reg     [1:0]                       cur_state;
    reg     [1:0]                       next_state;
    reg     [RBB_RD_ADDR_WIDTH-1:0]     rd_counter_d;
    reg     [RBB_RD_ADDR_WIDTH-1:0]     rd_counter;
    reg                                 TestCmp;

    localparam IDLE         = 'b01;
    localparam WRITE        = 'b10;
    localparam NUM_LINES    = (1 << RBB_RD_ADDR_WIDTH);

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
                if (ReqAck && rd_counter == (NUM_LINES-1))
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
    wire Full       = (cur_state == WRITE);
    wire Empty      = ~Full;
    wire ReqValid   = (cur_state == WRITE);
    
    reg                                 WrEn_BRAM;
    reg  [RBB_RD_DATA_WIDTH-1:0]        WrDin_BRAM;
    reg  [RBB_RD_ADDR_WIDTH-1:0]        WrAddr_BRAM;
    wire [3:0]                          WrAddr_Low  = WrAddr[3:0];

    always @ (posedge clk)
    begin
        if (~reset_n) begin
            WrEn_BRAM       <= 'b0;
            WrDin_BRAM      <= 'b0;
            WrAddr_BRAM     <= 'b0;
        end else begin
            WrDin_BRAM      <= {WrDin_BRAM[RBB_RD_DATA_WIDTH-RBB_WR_DATA_WIDTH-1:0], WrDin};
            WrAddr_BRAM     <= WrAddr[RBB_WR_ADDR_WIDTH-1:4];
            if (WrAddr_Low == 4'b1111) begin
                WrEn_BRAM   <= WrEn;
            end
        end
    end

    wire    [RBB_RD_ADDR_WIDTH-1:0]        RdAddrBRAM = (cur_state == IDLE && next_state == WRITE) ? 'b0 : rd_counter + 'b1;
    reg     [RBB_RD_ADDR_WIDTH-1:0]        RdAddrBRAM_r;
    
    always @ (posedge clk)
    begin
        RdAddrBRAM_r <= RdAddrBRAM;
    end

    wire    [RBB_RD_DATA_WIDTH-1:0]     RdDataBRAM;
    reg     [RBB_RD_ADDR_WIDTH-1:0]     ReqLineIdx;
    reg     [RBB_RD_ADDR_WIDTH-1:0]     ReqLineIdx_r;
    reg     [RBB_RD_DATA_WIDTH-1:0]     RdDout;
    reg     [RBB_RD_DATA_WIDTH-1:0]     RdDout_r;
    reg                                 task_done_r;
    reg                                 ReqAck_r;
    
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
        if (!reset_n) begin
            rd_counter  <= 0;
            TestCmp     <= 0;
        end else begin
            rd_counter <= rd_counter_d;
            if (   cur_state == WRITE 
                && next_state == IDLE 
                && rd_counter == (NUM_LINES-1))
                TestCmp <= 1;
        end 
    end

    always @ (*)
    begin
        rd_counter_d = rd_counter;
        if (cur_state == WRITE && ReqAck) 
        begin
            if (rd_counter == (NUM_LINES-1))
                rd_counter_d = 'b0;
            else
                rd_counter_d = rd_counter + 'b1;
        end
    end

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------
    nlb_gram_sdp #(.BUS_SIZE_ADDR(RBB_RD_ADDR_WIDTH),
                   .BUS_SIZE_DATA(RBB_RD_DATA_WIDTH),
                   .GRAM_MODE(2'd1)
    )rbb_bram 
    (
        .clk  (clk),
        .we   (WrEn_BRAM),
        .waddr(WrAddr_BRAM),
        .din  (WrDin_BRAM),
        .raddr(RdAddrBRAM),
        .dout (RdDataBRAM)
    );     

endmodule
