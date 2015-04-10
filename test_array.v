// ----------------------------------------------------------------------------
//						Loopback - memory copy test
// ----------------------------------------------------------------------------
//
// This is a memory copy test. It copies cache lines from source to destination 
// buffer.
//

module test_array #(parameter TBB_DATA_WIDTH=32, TBB_ADDR_WIDTH=16, RBB_DATA_WIDTH=512, RBB_ADDR_WIDTH=8)
(
    clk,
    reset_n,
    
    bm2pe_start,
    pe2bm_done,

    pe2bm_rbbWrEn,
    pe2bm_rbbWrAddr,
    pe2bm_rbbWrDin,
    
    pe2bm_tbbRdAddr,
    bm2pe_tbbRdDout
);

    input                           clk;
    input                           reset_n;

    input                           bm2pe_start;
    output                          pe2bm_done;

    output                          pe2bm_rbbWrEn;
    output  [RBB_ADDR_WIDTH-1:0]    pe2bm_rbbWrAddr;
    output  [RBB_DATA_WIDTH-1:0]    pe2bm_rbbWrDin;

    output  [TBB_ADDR_WIDTH-1:0]    pe2bm_tbbRdAddr;
    input   [TBB_DATA_WIDTH-1:0]    bm2pe_tbbRdDout;

    localparam  writeback       = 128'hdead_beef;
    localparam  NUM_WRITES      = 1 << RBB_ADDR_WIDTH;
    localparam  NUM_READS       = 1 << TBB_ADDR_WIDTH;

    reg                             rd_start;
    reg     [TBB_DATA_WIDTH-1:0]    rd_data;
    reg     [TBB_ADDR_WIDTH-1:0]    rd_counter;
    reg     [TBB_ADDR_WIDTH-1:0]    rd_counter_d;
    reg                             wr_start;
    reg     [RBB_DATA_WIDTH-1:0]    wr_data;
    reg     [RBB_ADDR_WIDTH-1:0]    wr_counter;
    reg     [RBB_ADDR_WIDTH-1:0]    wr_counter_d;
    reg                             wr_done;

    assign      pe2bm_rbbWrDin  = wr_data;
    assign      pe2bm_rbbWrEn   = wr_start;
    assign      pe2bm_done      = wr_done;
    assign      pe2bm_rbbWrAddr = wr_counter;
    assign      pe2bm_tbbRdAddr = rd_counter;

    always @ (posedge clk)
    begin
        if (~reset_n) begin
            wr_start    <= 'b0;
            wr_data     <= 'b0;
            wr_counter  <= 'b0;
            rd_start    <= 'b0;
            rd_counter  <= 'b0;
        end else begin
            wr_counter  <= wr_counter_d;
            rd_counter  <= rd_counter_d;
            if (bm2pe_start) begin
                rd_start    <= 'b1;
            end
            if (rd_counter == NUM_READS - 1) begin
                rd_start    <= 'b0;
                wr_start    <= 'b1;
                wr_data     <= writeback;
            end
        end
    end

    always @ (*)
    begin
        wr_counter_d = wr_counter;
        rd_counter_d = rd_counter;
        if (wr_start) begin
            if (wr_counter == NUM_WRITES - 1) begin
                wr_counter_d    = 'b0;
                wr_done         = 'b1;
            end else begin
                wr_counter_d    = wr_counter + 'b1;
            end
        end else begin
            wr_done = 'b0;
        end
        if (rd_start) begin
            if (rd_counter == NUM_READS - 1) begin
                rd_counter_d    = 'b0;
            end else begin
                rd_counter_d    = rd_counter + 'b1;
            end
        end
    end

    always @ (posedge clk)
    begin
        if (~reset_n) begin
            rd_data <= 0;
        end else begin
            rd_data <= bm2pe_tbbRdDout;
        end
    end

endmodule
