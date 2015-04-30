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
    wire [3:0]                          WrAddr_Low  = WrAddr[3:0];
    reg  [RBB_WR_ADDR_WIDTH-5:0]        WrAddr_BRAM;
    reg  [4:0]                          WrDin_idx;     // FIXME

    // glue 16 32-bit inputs into a 512-bit input
    always @ (posedge clk)
    begin
        if (~reset_n) begin
            WrEn_BRAM       <= 'b0;
            WrDin_BRAM      <= 'b0;
            WrAddr_BRAM     <= 'b0;
            WrDin_idx       <= 'b0;
        end else begin
            if (WrEn) begin
                if (WrDin_idx == 4'hF) begin
                    WrDin_idx <= 4'h0;
                end else begin
                    WrDin_idx <= WrDin_idx + 'b1;
                end
                case (WrDin_idx)    /* synthesis parallel_case */
                    4'h0: WrDin_BRAM[RBB_WR_DATA_WIDTH*16-1:RBB_WR_DATA_WIDTH*15]   <= WrDin;
                    4'h1: WrDin_BRAM[RBB_WR_DATA_WIDTH*15-1:RBB_WR_DATA_WIDTH*14]   <= WrDin;
                    4'h2: WrDin_BRAM[RBB_WR_DATA_WIDTH*14-1:RBB_WR_DATA_WIDTH*13]   <= WrDin;
                    4'h3: WrDin_BRAM[RBB_WR_DATA_WIDTH*13-1:RBB_WR_DATA_WIDTH*12]   <= WrDin;
                    4'h4: WrDin_BRAM[RBB_WR_DATA_WIDTH*12-1:RBB_WR_DATA_WIDTH*11]   <= WrDin;
                    4'h5: WrDin_BRAM[RBB_WR_DATA_WIDTH*11-1:RBB_WR_DATA_WIDTH*10]   <= WrDin;
                    4'h6: WrDin_BRAM[RBB_WR_DATA_WIDTH*10-1:RBB_WR_DATA_WIDTH*9]    <= WrDin;
                    4'h7: WrDin_BRAM[RBB_WR_DATA_WIDTH*9 -1:RBB_WR_DATA_WIDTH*8]    <= WrDin;
                    4'h8: WrDin_BRAM[RBB_WR_DATA_WIDTH*8 -1:RBB_WR_DATA_WIDTH*7]    <= WrDin;
                    4'h9: WrDin_BRAM[RBB_WR_DATA_WIDTH*7 -1:RBB_WR_DATA_WIDTH*6]    <= WrDin;
                    4'hA: WrDin_BRAM[RBB_WR_DATA_WIDTH*6 -1:RBB_WR_DATA_WIDTH*5]    <= WrDin;
                    4'hB: WrDin_BRAM[RBB_WR_DATA_WIDTH*5 -1:RBB_WR_DATA_WIDTH*4]    <= WrDin;
                    4'hC: WrDin_BRAM[RBB_WR_DATA_WIDTH*4 -1:RBB_WR_DATA_WIDTH*3]    <= WrDin;
                    4'hD: WrDin_BRAM[RBB_WR_DATA_WIDTH*3 -1:RBB_WR_DATA_WIDTH*2]    <= WrDin;
                    4'hE: WrDin_BRAM[RBB_WR_DATA_WIDTH*2 -1:RBB_WR_DATA_WIDTH*1]    <= WrDin;
                    4'hF: WrDin_BRAM[RBB_WR_DATA_WIDTH   -1:0]                      <= WrDin;
                endcase
            end
            if (WrEn && WrAddr_Low == 4'b1111) begin
                WrEn_BRAM   <= 'b1;
                WrAddr_BRAM <= WrAddr[RBB_WR_ADDR_WIDTH-1:4];
            end else if (task_done && WrAddr_BRAM != {RBB_WR_DATA_WIDTH{1'b1}}) begin
                WrEn_BRAM   <= 'b1;
                WrAddr_BRAM <= WrAddr_BRAM + 'b1;
            end else begin
                WrEn_BRAM   <= 'b0;
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
            else 
                TestCmp <= 0;
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
