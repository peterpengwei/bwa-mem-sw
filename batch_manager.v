// ***************************************************************************
//
//        Batch Manager
//
// Engineer:            Peng Wei
// Create Date:         Feb 12, 2015
// Module Name:         batch_manager
// Description:         manage each PE array's task batch buffer and result
//                      batch buffer
//                      it instantiates TBBs and RBBs
// ***************************************************************************
//
// CSR Address Map -- Change v1.1
//------------------------------------------------------------------------------------------
//      Address[15:0] Attribute         Name                    Comments
//     'h1A00          WO                CSR_AFU_DSM_BASEL       Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
//     'h1A04          WO                CSR_AFU_DSM_BASEH       Upper 32-bits of AFU DSM base address.
//     'h1A20:         WO                CSR_SRC_ADDR            Start physical address for source buffer. All read requests are targetted to this region.
//     'h1A24:         WO                CSR_DST_ADDR            Start physical address for destination buffer. All write requests are targetted to this region.
//     'h1A28:         WO                CSR_NUM_BATCHES           Number of cache lines
//     'h1A2c:         WO                CSR_CTL                 Controls test flow, start, stop, force completion
//     'h1A34:         WO                CSR_CFG                 Configures test parameters
//     'h1A38:         WO                CSR_INACT_THRESH        inactivity threshold limit
//     'h1A3c          WO                CSR_INTERRUPT0          SW allocates Interrupt APIC ID & Vector to device
//     
//
// DSM Offeset Map -- Change v1.1
//------------------------------------------------------------------------------------------
//      Byte Offset   Attribute         Name                  Comments
//      0x00          RO                DSM_AFU_ID            non-zero value to uniquely identify the AFU
//      0x40          RO                DSM_STATUS            test status and error register
//
//
// 1 Cacheline = 64B i.e 2^6 Bytes
// Let N be the number of cachelines in the source & destination buffers. Then select CSR_SRC_ADDR & CSR_DEST_ADDR to be 2^(N+6) aligned.
// CSR_NUM_BATCHES should be less than or equal to N.
//
// CSR_SRC_ADDR:
// [31:N]   WO   2^(N+6)MB aligned address points to the start of read buffer
// [N-1:0]  WO   'h0
//
// CSR_DST_ADDR:
// [31:N]   WO   2^(N+6)MB aligned address points to the start of write buffer
// [N-1:0]  WO   'h0
//
// CSR_NUM_BATCHES:
// [31:N]   WO   'h0
// [N-1:0]  WO    # cache lines to be read/written to. This threshold may be different for each test AFU. IMPORTANT- Ensure that source and destination buffers 
//              are large enough to accomodate the N cache lines.
//
// Let's assume N=14, then CSR_SRC_ADDR and CSR_DST_ADDR will accept a 2^20, i.e. 1MB aligned addresses.
//
// CSR_SRC_ADDR:
// [31:14]  WO    1MB aligned address
// [13:0]   WO   'h0
//
// CSR_DST_ADDR:
// [31:14]  WO    1MB aligned address
// [13:0]   WO   'h0
//
// CSR_NUM_BATCHES:
// [31:14]  WO    'h0
// [13:0]   WO    # cache lines to be read/written to. This threshold may be different for each test AFU. IMPORTANT- Ensure that source and destination buffers 
//              are large enough to accomodate the # cache lines.
//
// CSR_CTL:
// [31:3]   WO    Rsvd
// [2]      WO    Force test completion. Writes test completion flag and other performance counters to csr_stat. It appears to be like a normal test completion.
// [1]      WO    Starts test execution.
// [0]      WO    Active low test Reset. All configuration parameters change to reset defaults.
//
//
// CSR_CFG:
// [29]     WO    cr_interrupt_testmode - used to test interrupt. Generates an interrupt at end of each test.
// [28]     WO    cr_interrupt_on_error - send an interrupt when error detected
// [27:20]  WO    cr_test_cfg  -may be used to configure the behavior of each test mode
// [10:9]   WO    cr_rdsel     -configure read request type. 0- RdLine_S, 1- RdLine_I, 2- RdLine_O, 3- Mixed mode
// [8]      WO    cr_delay_en  -enable random delay insertion between requests
// [4:2]    WO    cr_mode      -configures test mode
// [1]      WO    cr_cont      - 1- test rollsover to start address after it reaches the CSR_NUM_BATCHES count. Such a test terminates only on an error.
//                               0- test terminates, updated the status csr when CSR_NUM_BATCHES count is reached.
// [0]      WO    cr_wrthru_en -switch between write back to write through request type. 0- Wr Back, 1- WrThru
// 
//
// CSR_INACT_THRESHOLD:
// [31:0]   WO  inactivity threshold limit. The idea is to detect longer duration of stalls during a test run. Inactivity counter will count number of consecutive idle cycles,
//              i.e. no requests are sent and no responses are received. If the inactivity count > CSR_INACT_THRESHOLD then it sets the inact_timeout signal. The inactivity counter
//              is activated only after test is started by writing 1 to CSR_CTL[1].
//
// CSR_INTERRUPT0:
// [23:16]  WO    vector      - Interrupt Vector # for the device
// [15:0]   WO    apic id     - Interrupt APIC ID for the device 
//
// DSM_STATUS:
// [511:256] RO  Error dump from Test Mode
// [255:224] RO  end overhead
// [223:192] RO  start overhead
// [191:160] RO  Number of writes
// [159:128] RO  Number of reads
// [127:64]  RO  Number of clocks
// [63:32]   RO  test error register
// [31:0]    RO  test completion flag
//
// DSM_AFU_ID:
// [512:144] RO   Zeros
// [143:128] RO   Version
// [127:0]   RO   AFU ID 

module batch_manager #(parameter    TBB_WR_ADDR_WIDTH=12, 
                                    TBB_WR_DATA_WIDTH=512, 
                                    TBB_RD_ADDR_WIDTH=16,
                                    TBB_RD_DATA_WIDTH=32,
                                    RBB_WR_ADDR_WIDTH=12,
                                    RBB_WR_DATA_WIDTH=32,
                                    RBB_RD_ADDR_WIDTH=8,  
                                    RBB_RD_DATA_WIDTH=512, 
                                    NUM_PEA=4,
                                    TXHDR_WIDTH=61,
                                    RXHDR_WIDTH=61,
                                    DATA_WIDTH=512
                                    )
(
    // ---------------------------global signals-------------------------------------------------
    clk,                              //              in    std_logic;  -- Core clock
    reset_n,                          //              in    std_logic;  -- Use SPARINGLY only for control
    // ---------------------------IF signals between SPL and FPL  --------------------------------
    rb2cf_C0RxHdr,                    // [RXHDR_WIDTH-1:0]   cci_intf:           Rx header to SPL channel 0
    rb2cf_C0RxData,                   // [DATA_WIDTH -1:0]   cci_intf:           Rx data response to SPL | no back pressure
    rb2cf_C0RxWrValid,                //                     cci_intf:           Rx write response enable
    rb2cf_C0RxRdValid,                //                     cci_intf:           Rx read response enable
    rb2cf_C0RxCfgValid,               //                     cci_intf:           Rx config response enable
    //rb2cf_C0RxUMsgValid,            //                     cci_intf:           Rx UMsg valid
    //rb2cf_C0RxIntrValid,            //                     cci_intf:           Rx interrupt valid
    rb2cf_C1RxHdr,                    // [RXHDR_WIDTH-1:0]   cci_intf:           Rx header to SPL channel 1
    rb2cf_C1RxWrValid,                //                     cci_intf:           Rx write response valid
    //rb2cf_C1RxIntrValid,            //                     cci_intf:           Rx interrupt valid

    cf2ci_C0TxHdr,                    // [TXHDR_WIDTH-1:0]   cci_intf:           Tx Header from SPL channel 0
    cf2ci_C0TxRdValid,                //                     cci_intf:           Tx read request enable
    cf2ci_C1TxHdr,                    //                     cci_intf:           Tx Header from SPL channel 1
    cf2ci_C1TxData,                   //                     cci_intf:           Tx data from SPL
    cf2ci_C1TxWrValid,                //                     cci_intf:           Tx write request enable
    //cf2ci_C1TxIntrValid,            //                     cci_intf:           Tx interrupt valid
    ci2cf_C0TxAlmFull,                //                     cci_intf:           Tx memory channel 0 almost full
    ci2cf_C1TxAlmFull,                //                     cci_intf:           TX memory channel 1 almost full

    ci2cf_InitDn                      // Link initialization is complete
);

    input                        clk;                  //              in    std_logic;  -- Core clock
    input                        reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control

    input [RXHDR_WIDTH-1:0]      rb2cf_C0RxHdr;        // [RXHDR_WIDTH-1:0]cci_intf:           Rx header to SPL channel 0
    input [DATA_WIDTH -1:0]      rb2cf_C0RxData;       // [DATA_WIDTH -1:0]cci_intf:           data response to SPL | no back pressure
    input                        rb2cf_C0RxWrValid;    //                  cci_intf:           write response enable
    input                        rb2cf_C0RxRdValid;    //                  cci_intf:           read response enable
    input                        rb2cf_C0RxCfgValid;   //                  cci_intf:           config response enable
    // input                        rb2cf_C0RxUMsgValid;  //                  cci_intf:           Rx UMsg valid
    // input                        rb2cf_C0RxIntrValid;  //                  cci_intf:           interrupt response enable
    input [RXHDR_WIDTH-1:0]      rb2cf_C1RxHdr;        // [RXHDR_WIDTH-1:0]cci_intf:           Rx header to SPL channel 1
    input                        rb2cf_C1RxWrValid;    //                  cci_intf:           write response valid
    // input                        rb2cf_C1RxIntrValid;  //                  cci_intf:           interrupt response valid

    output [TXHDR_WIDTH-1:0]     cf2ci_C0TxHdr;        // [TXHDR_WIDTH-1:0]cci_intf:           Tx Header from SPL channel 0
    output                       cf2ci_C0TxRdValid;    //                  cci_intf:           Tx read request enable
    output [TXHDR_WIDTH-1:0]     cf2ci_C1TxHdr;        //                  cci_intf:           Tx Header from SPL channel 1
    output [DATA_WIDTH -1:0]     cf2ci_C1TxData;       //                  cci_intf:           Tx data from SPL
    output                       cf2ci_C1TxWrValid;    //                  cci_intf:           Tx write request enable
    // output                       cf2ci_C1TxIntrValid;  //                  cci_intf:           Tx interrupt valid
    input                        ci2cf_C0TxAlmFull;    //                  cci_intf:           Tx memory channel 0 almost full
    input                        ci2cf_C1TxAlmFull;    //                  cci_intf:           TX memory channel 1 almost full

    input                        ci2cf_InitDn;         //                  cci_intf:           Link initialization is complete

    //----------------------------------------------------------------------------------------------------------------------
    // BWA_MEM_SW AFU ID
    // It is important to keep the least significant 4 bits NON-ZERO to be compliant with CCIDemo.cpp
    //
    localparam       BWA_MEM_SW          = 128'h2015_0408_900d_beef_a000_b000_c000_d000;
    localparam       VERSION             = 16'h0001;
    
    //---------------------------------------------------------
    // CCI-S Request Encodings  ***** DO NOT MODIFY ******
    //---------------------------------------------------------
    localparam       WrThru              = 4'h1;
    localparam       WrLine              = 4'h2;
    localparam       RdLine              = 4'h4;
    localparam       WrFence             = 4'h5;
    
    //--------------------------------------------------------
    // CCI-S Response Encodings  ***** DO NOT MODIFY ******
    //--------------------------------------------------------
    localparam      RSP_CSR              = 4'h0;
    localparam      RSP_WRITE            = 4'h1;
    localparam      RSP_READ             = 4'h4;
    
    //---------------------------------------------------------
    // Default Values ****** May be MODIFIED ******* 
    //---------------------------------------------------------
    localparam      DEF_SRC_ADDR         = 32'h0400_0000;           // Read data starting from here. Cache aligned Address
    localparam      DEF_DST_ADDR         = 32'h0500_0000;           // Copy data to here. Cache aligned Address

    localparam      DEF_DSM_BASE         = 32'h04ff_ffff;           // default status address
    
    //---------------------------------------------------------
    // CSR Address Map ***** DO NOT MODIFY *****
    //---------------------------------------------------------
    localparam      CSR_AFU_DSM_BASEL    = 16'h1a00;                 // WO - Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
    localparam      CSR_AFU_DSM_BASEH    = 16'h1a04;                 // WO - Upper 32-bits of AFU DSM base address.
    localparam      CSR_SRC_ADDR         = 16'h1a20;                 // WO   Reads are targetted to this region 
    localparam      CSR_DST_ADDR         = 16'h1a24;                 // WO   Writes are targetted to this region
    localparam      CSR_REQ_PEARRAY      = 16'h1a28;                 // WO   Numbers of task batches to be read/write
    localparam      CSR_CTL              = 16'h1a2c;                 // WO   Control CSR to start n stop the test
    
    //----------------------------------------------------------------------------------
    // Device Status Memory (DSM) Address Map ***** DO NOT MODIFY *****
    // Physical address = value at CSR_AFU_DSM_BASE + Byte offset
    //----------------------------------------------------------------------------------
    //                                     Byte Offset                 Attribute    Width   Comments
    localparam      DSM_AFU_ID           = 32'h0;                   // RO           32b     non-zero value to uniquely identify the AFU
    localparam      DSM_STATUS           = 32'h40;                  // RO           512b    test status and error info
    localparam      POINTER_WIDTH  = NUM_PEA<=2 ? 1 : (NUM_PEA<=4 ? 2 : -1);
    localparam      RD_REQ_POINTER_WIDTH = 14 - TBB_WR_ADDR_WIDTH;
    
    //----------------------------------------------------------------------------------------------------------------------

    localparam      RDID_ZEROS    = 14 - TBB_WR_ADDR_WIDTH - 2;
    localparam      WRID_ZEROS    = 14 - RBB_RD_ADDR_WIDTH - 2;

    wire [NUM_PEA-1:0]                      bm2pe_start_b;
    wire [NUM_PEA-1:0]                      pe2bm_done_b;
  
    reg  [NUM_PEA-1:0]                      tbbWrEn_b;
    reg  [TBB_WR_ADDR_WIDTH*NUM_PEA-1:0]    tbbWrAddr_b;
    reg  [TBB_WR_DATA_WIDTH*NUM_PEA-1:0]    tbbWrDin_b;
    wire [NUM_PEA-1:0]                      tbbFull_b;
    wire [TBB_RD_ADDR_WIDTH*NUM_PEA-1:0]    tbbRdAddr_b;
    wire [TBB_RD_DATA_WIDTH*NUM_PEA-1:0]    tbbRdDout_b;
    wire [NUM_PEA-1:0]                      tbbEmpty_b;
    wire [NUM_PEA-1:0]                      tbbReqValid_b;
    wire [NUM_PEA*TBB_WR_ADDR_WIDTH-1:0]    tbbReqLineIdx_b;
    reg  [NUM_PEA-1:0]                      tbbReqAck_b;

    wire [NUM_PEA-1:0]                      rbbWrEn_b;
    wire [RBB_WR_ADDR_WIDTH*NUM_PEA-1:0]    rbbWrAddr_b;
    wire [RBB_WR_DATA_WIDTH*NUM_PEA-1:0]    rbbWrDin_b;
    wire [NUM_PEA-1:0]                      rbbFull_b;
    wire [RBB_RD_ADDR_WIDTH*NUM_PEA-1:0]    rbbRdAddr_b;
    wire [RBB_RD_DATA_WIDTH*NUM_PEA-1:0]    rbbRdDout_b;
    wire [NUM_PEA-1:0]                      rbbEmpty_b;
    wire [NUM_PEA-1:0]                      rbbReqValid_b;
    wire [NUM_PEA*RBB_RD_ADDR_WIDTH-1:0]    rbbReqLineIdx_b;
    reg  [NUM_PEA-1:0]                      rbbReqAck_b;
    wire [NUM_PEA-1:0]                      rbbTestCmp_b;
    wire                                    bm_TestCmp;

    assign bm_TestCmp             = |rbbTestCmp_b;          // Write to DSM reg signal 
    
    reg     [DATA_WIDTH-1:0]        cf2ci_C1TxData;
    reg     [TXHDR_WIDTH-1:0]       cf2ci_C1TxHdr;
    reg                             cf2ci_C1TxWrValid;
    reg     [TXHDR_WIDTH-1:0]       cf2ci_C0TxHdr;
    reg                             cf2ci_C0TxRdValid;
    
    reg                             dsm_base_valid;
    reg                             dsm_base_valid_q;
    reg                             afuid_updtd;
    reg                             status_updtd;
    reg                             status_updtd_d;
    
    reg     [63:0]                  cr_dsm_base;            // a00h, a04h - DSM base address
    reg     [31:0]                  cr_src_address;         // a20h - source buffer address
    reg     [31:0]                  cr_dst_address;         // a24h - destn buffer address
    reg     [31:0]                  cr_request_pearray;     // a28h - software requests for pearrays
    reg     [31:0]                  cr_ctl  = 0;            // a2ch - control register to start and stop the test
    wire                            test_go = cr_ctl[1];    // When 0, it allows reconfiguration of test parameters.

    //register for storing number of task batches that get received
    reg     [NUM_PEA-1:0]           pearray_busy;
    reg     [NUM_PEA-1:0]           pearray_busy_d;
    //pointer to TBB
    reg     [POINTER_WIDTH-1:0]     tbb_pointer;
    reg     [POINTER_WIDTH-1:0]     tbb_pointer_d;
    //pointer to RBB
    reg     [POINTER_WIDTH-1:0]     rbb_pointer;
    reg     [POINTER_WIDTH-1:0]     rbb_pointer_d;

    //CCI Read Address Offset
    reg     [TBB_WR_ADDR_WIDTH-1:0] RdAddrOffset;
    reg     [TBB_WR_ADDR_WIDTH-1:0] RdAddrOffset_d;
    //CCI Read ID
    reg     [13:0]                  RdReqId;
    //CCI Read Type
    wire    [3:0]                   rdreq_type = RdLine;

    //CCI Write Address Offset
    reg     [RBB_RD_ADDR_WIDTH-1:0] WrAddrOffset;
    reg     [RBB_RD_ADDR_WIDTH-1:0] WrAddrOffset_d;
    //CCI Write ID
    reg     [13:0]                  WrReqId;
    //CCI Write Type
    wire    [3:0]                   wrreq_type = WrLine;

    wire    [31:0]                  ds_afuid_address = dsm_offset2addr(DSM_AFU_ID,cr_dsm_base);     // 0h - afu id is written to this address
    wire    [31:0]                  ds_stat_address = dsm_offset2addr(DSM_STATUS,cr_dsm_base);      // 40h - test status is written to this address
    wire                            re2xy_go = test_go & afuid_updtd & ci2cf_InitDn;                // After initializing DSM, we can do actual tasks on AFU
    reg                             WrHdr_valid;                                                    // 1: Valid Write Request
    reg                             RdHdr_valid;                                                    // 1: Valid Read Request

    //-------------------------
    //CSR Register Handling
    //-------------------------
    always @(posedge clk)                                              
    begin                                                                   
        if(!reset_n)
        begin
            cr_dsm_base     <= DEF_DSM_BASE;
            cr_src_address  <= DEF_SRC_ADDR;
            cr_dst_address  <= DEF_DST_ADDR;
            cr_ctl          <= 'b0;
            dsm_base_valid  <= 'b0;
        end
        else
        begin  
        //control register can be written anytime after resetting
            if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
                    CSR_CTL          :   cr_ctl             <= rb2cf_C0RxData[31:0];
                endcase
            if(~test_go) // Configuration Mode, following CSRs can only be updated in this mode
            begin
                if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
                    CSR_SRC_ADDR:        cr_src_address     <= rb2cf_C0RxData[31:0];
                    CSR_DST_ADDR:        cr_dst_address     <= rb2cf_C0RxData[31:0];
                    CSR_AFU_DSM_BASEH:   cr_dsm_base[63:32] <= rb2cf_C0RxData[31:0];
                    CSR_AFU_DSM_BASEL:begin
                                         cr_dsm_base[31:0]  <= rb2cf_C0RxData[31:0];
                                         dsm_base_valid     <= 'b1;
                                      end
                endcase
            end
            if(re2xy_go) // Execution Mode, following CSRs can only be updated in this mode
            begin
                if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
                    CSR_REQ_PEARRAY:    cr_request_pearray  <= rb2cf_C0RxData[31:0];
                endcase
            end
        end
    end

    //-------------------------
    // Round-Robin Load/Store
    //-------------------------

    // Sequential Logic
    always @ (posedge clk) 
    begin
        if (!reset_n)
        begin
            tbb_pointer     <= 'b0;
            rbb_pointer     <= 'b0;
            RdAddrOffset    <= 'b0;
            WrAddrOffset    <= 'b0;
            pearray_busy    <= 'b0;
        end
        else
        begin
            tbb_pointer     <= tbb_pointer_d;
            rbb_pointer     <= rbb_pointer_d;
            RdAddrOffset    <= RdAddrOffset_d;
            WrAddrOffset    <= WrAddrOffset_d;
            pearray_busy    <= pearray_busy_d;
        end
    end

    // Combinatorial Logic
    always @ (*) 
    begin
        tbb_pointer_d       = tbb_pointer;
        rbb_pointer_d       = rbb_pointer;
        RdAddrOffset_d      = RdAddrOffset;
        WrAddrOffset_d      = WrAddrOffset;
        pearray_busy_d      = pearray_busy;
        RdHdr_valid         = 'b0;
        WrHdr_valid         = 'b0;
        tbbReqAck_b         = 'b0;
        rbbReqAck_b         = 'b0;
        RdReqId             = 'b0;
        WrReqId             = 'b0;
        status_updtd_d      = status_updtd;
      
        if (re2xy_go) //During the real execution state, do the modification on these registers
        begin
            //tbb handler
            case(tbb_pointer)         /* synthesis parallel_case */
                'b0:
                begin
                    //If current tbb is full, jump it without doing anything
                    if (tbbFull_b[0])
                    begin
                        if (tbb_pointer == (NUM_PEA-1))
                            tbb_pointer_d = 'b0;
                        else
                            tbb_pointer_d = tbb_pointer + 'b1;
                    end
                    //If tbb is not full, it can store a valid task
                    //Then, check if there is a valid task to process
                    else if (cr_request_pearray[0])
                    begin
                        //We have at least one available batch to process
                        //Then, check if the TBB is willing to read data
                        if (tbbReqValid_b[0] && !ci2cf_C0TxAlmFull) //Not full, Available Batch, Request Valid and CCI not stalled, then just fetch data
                        begin
                            RdHdr_valid = 'b1; // a valid read request, 100% sent
                            RdAddrOffset_d = RdAddrOffset + 'b1; // update address since the current one has been sent
                            RdReqId = {{RDID_ZEROS{1'b0}}, 2'b0, tbbReqLineIdx_b[0*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:0*TBB_WR_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            tbbReqAck_b[0] = 'b1; // tell the TBB that the request has been sent to CCI
                        end
                        //if TBB is willing to read data but CCI is stalled, then do nothing
                        else if (tbbReqValid_b[0])
                        begin
                            //do nothing
                        end
                        //if TBB is not willing to read data, then we can infer that the TBB has sent all the read requests
                        //We can go to next TBB, and meanwhile claim that a batch has been received (since all the requests of a batch have been sent to CCI)
                        else
                        begin
                            if (tbb_pointer == (NUM_PEA-1))
                                tbb_pointer_d = 'b0;
                            else
                                tbb_pointer_d = tbb_pointer + 'b1;
                            pearray_busy_d[0] = 1'b1;
                            status_updtd_d    = 1'b0;
                        end
                    end
                end
                'b1:
                begin
                    //If current tbb is full, jump it without doing anything
                    if (tbbFull_b[1])
                    begin
                        if (tbb_pointer == (NUM_PEA-1))
                            tbb_pointer_d = 'b0;
                        else
                            tbb_pointer_d = tbb_pointer + 'b1;
                    end
                    //If tbb is not full, it can store a valid task
                    //Then, check if there is a valid task to process
                    else if (cr_request_pearray[1])
                    begin
                        //We have at least one available batch to process
                        //Then, check if the TBB is willing to read data
                        if (tbbReqValid_b[1] && !ci2cf_C0TxAlmFull) //Not full, Available Batch, Request Valid and CCI not stalled, then just fetch data
                        begin
                            RdHdr_valid = 'b1; // a valid read request, 100% sent
                            RdAddrOffset_d = RdAddrOffset + 'b1; // update address since the current one has been sent
                            RdReqId = {{RDID_ZEROS{1'b0}}, 2'b1, tbbReqLineIdx_b[1*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:1*TBB_WR_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            tbbReqAck_b[1] = 'b1; // tell the TBB that the request has been sent to CCI
                        end
                        //if TBB is willing to read data but CCI is stalled, then do nothing
                        else if (tbbReqValid_b[1])
                        begin
                            //do nothing
                        end
                    //if TBB is not willing to read data, then we can infer that the TBB has sent all the read requests
                    //We can go to next TBB, and meanwhile claim that a batch has been received (since all the requests of a batch have been sent to CCI)
                        else
                        begin
                            if (tbb_pointer == (NUM_PEA-1))
                                tbb_pointer_d = 'b0;
                            else
                                tbb_pointer_d = tbb_pointer + 'b1;
                            pearray_busy_d[1] = 1'b1;
                            status_updtd_d    = 1'b0;
                        end
                    end
                end
                'b10:
                begin
                    //If current tbb is full, jump it without doing anything
                    if (tbbFull_b[2])
                    begin
                        if (tbb_pointer == (NUM_PEA-1))
                            tbb_pointer_d = 'b0;
                        else
                            tbb_pointer_d = tbb_pointer + 'b1;
                    end
                    //If tbb is not full, it can store a valid task
                    //Then, check if there is a valid task to process
                    else if (cr_request_pearray[2])
                    begin
                        //We have at least one available batch to process
                        //Then, check if the TBB is willing to read data
                        if (tbbReqValid_b[2] && !ci2cf_C0TxAlmFull) //Not full, Available Batch, Request Valid and CCI not stalled, then just fetch data
                        begin
                            RdHdr_valid = 'b1; // a valid read request, 100% sent
                            RdAddrOffset_d = RdAddrOffset + 'b1; // update address since the current one has been sent
                            RdReqId = {{RDID_ZEROS{1'b0}}, 2'b10, tbbReqLineIdx_b[2*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:2*TBB_WR_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            tbbReqAck_b[2] = 'b1; // tell the TBB that the request has been sent to CCI
                        end
                        //if TBB is willing to read data but CCI is stalled, then do nothing
                        else if (tbbReqValid_b[2])
                        begin
                            //do nothing
                        end
                        //if TBB is not willing to read data, then we can infer that the TBB has sent all the read requests
                        //We can go to next TBB, and meanwhile claim that a batch has been received (since all the requests of a batch have been sent to CCI)
                        else
                        begin
                            if (tbb_pointer == (NUM_PEA-1))
                                tbb_pointer_d = 'b0;
                            else
                                tbb_pointer_d = tbb_pointer + 'b1;
                            pearray_busy_d[2] = 1'b1;
                            status_updtd_d    = 1'b0;
                        end
                    end
                end
                'b11:
                begin
                    //If current tbb is full, jump it without doing anything
                    if (tbbFull_b[3])
                    begin
                        if (tbb_pointer == (NUM_PEA-1))
                            tbb_pointer_d = 'b0;
                        else
                            tbb_pointer_d = tbb_pointer + 'b1;
                    end
                    //If tbb is not full, it can store a valid task
                    //Then, check if there is a valid task to process
                    else if (cr_request_pearray[3])
                    begin
                        //We have at least one available batch to process
                        //Then, check if the TBB is willing to read data
                        if (tbbReqValid_b[3] && !ci2cf_C0TxAlmFull) //Not full, Available Batch, Request Valid and CCI not stalled, then just fetch data
                        begin
                            RdHdr_valid = 'b1; // a valid read request, 100% sent
                            RdAddrOffset_d = RdAddrOffset + 'b1; // update address since the current one has been sent
                            RdReqId = {{RDID_ZEROS{1'b0}}, 2'b11, tbbReqLineIdx_b[3*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:3*TBB_WR_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            tbbReqAck_b[3] = 'b1; // tell the TBB that the request has been sent to CCI
                        end
                        //if TBB is willing to read data but CCI is stalled, then do nothing
                        else if (tbbReqValid_b[3])
                        begin
                            //do nothing
                        end
                        //if TBB is not willing to read data, then we can infer that the TBB has sent all the read requests
                        //We can go to next TBB, and meanwhile claim that a batch has been received (since all the requests of a batch have been sent to CCI)
                        else
                        begin
                            if (tbb_pointer == (NUM_PEA-1))
                                tbb_pointer_d = 'b0;
                            else
                                tbb_pointer_d = tbb_pointer + 'b1;
                            pearray_busy_d[3] = 1'b1;
                            status_updtd_d    = 1'b0;
                        end
                    end
                end
            endcase

            //-----------------------
            // RBB round-robin
            //-----------------------
            //rbb handler
            case(rbb_pointer)         /* synthesis parallel_case */
                'b0:
                begin
                    //If current rbb is empty, jump it without doing anything
                    if (rbbEmpty_b[0])
                    begin
                        if (rbb_pointer == (NUM_PEA-1))
                            rbb_pointer_d = 'b0;
                        else
                            rbb_pointer_d = rbb_pointer + 'b1;
                    end
                    //If current rbb is not empty, we are able to send data back to CCI
                    else
                    begin
                      //check if the RBB is willing to write data
                        if (rbbReqValid_b[0] && !ci2cf_C1TxAlmFull && status_updtd) //Not empty, Request Valid and CCI not stalled, then just write back data
                        begin
                            WrHdr_valid = 'b1; // a valid write request, 100% sent
                            WrAddrOffset_d = WrAddrOffset + 'b1; // update address since the current one has been sent
                            WrReqId = {{WRID_ZEROS{1'b0}}, 2'd0, rbbReqLineIdx_b[0*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:0*RBB_RD_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            rbbReqAck_b[0] = 'b1; // tell the RBB that the request has been sent to CCI
                        end
                        //if RBB is willing to write data but CCI is stalled, then do nothing
                        else if (rbbReqValid_b[0])
                        begin
                            //do nothing
                        end
                        //if RBB is not willing to write data, then we can infer that the RBB has sent all the write requests
                        //We can go to next RBB, and meanwhile claim that a result batch has been sent back
                        //Question: How to let CPU know it?
                        else
                        begin
                            if (rbb_pointer == (NUM_PEA-1))
                                rbb_pointer_d = 'b0;
                            else
                                rbb_pointer_d = rbb_pointer + 'b1;
                        end
                    end
                    if (rbbTestCmp_b[0]) begin
                        pearray_busy_d[0] = 1'b0;
                        status_updtd_d    = 1'b0;
                    end
                end
                'b1:
                begin
                    //If current rbb is empty, jump it without doing anything
                    if (rbbEmpty_b[1])
                    begin
                        if (rbb_pointer == (NUM_PEA-1))
                            rbb_pointer_d = 'b0;
                        else
                            rbb_pointer_d = rbb_pointer + 'b1;
                    end
                    //If current rbb is not empty, we are able to send data back to CCI
                    else
                    begin
                        //check if the RBB is willing to write data
                        if (rbbReqValid_b[1] && !ci2cf_C1TxAlmFull && status_updtd) //Not empty, Request Valid and CCI not stalled, then just write back data
                        begin
                            WrHdr_valid = 'b1; // a valid write request, 100% sent
                            WrAddrOffset_d = WrAddrOffset + 'b1; // update address since the current one has been sent
                            WrReqId = {{WRID_ZEROS{1'b0}}, 2'd1, rbbReqLineIdx_b[1*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:1*RBB_RD_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            rbbReqAck_b[1] = 'b1; // tell the RBB that the request has been sent to CCI
                        end
                        //if RBB is willing to write data but CCI is stalled, then do nothing
                        else if (rbbReqValid_b[1])
                        begin
                            //do nothing
                        end
                        //if RBB is not willing to write data, then we can infer that the RBB has sent all the write requests
                        //We can go to next RBB, and meanwhile claim that a result batch has been sent back
                        //Question: How to let CPU know it?
                        else
                        begin
                            if (rbb_pointer == (NUM_PEA-1))
                                rbb_pointer_d = 'b0;
                            else
                                rbb_pointer_d = rbb_pointer + 'b1;
                        end
                    end
                    if (rbbTestCmp_b[1]) begin
                        pearray_busy_d[1] = 1'b0;
                        status_updtd_d    = 1'b0;
                    end
                end
                'b10:
                begin
                    //If current rbb is empty, jump it without doing anything
                    if (rbbEmpty_b[2])
                    begin
                        if (rbb_pointer == (NUM_PEA-1))
                            rbb_pointer_d = 'b0;
                        else
                            rbb_pointer_d = rbb_pointer + 'b1;
                    end
                    //If current rbb is not empty, we are able to send data back to CCI
                    else
                    begin
                        //check if the RBB is willing to write data
                        if (rbbReqValid_b[2] && !ci2cf_C1TxAlmFull && status_updtd) //Not empty, Request Valid and CCI not stalled, then just write back data
                        begin
                            WrHdr_valid = 'b1; // a valid write request, 100% sent
                            WrAddrOffset_d = WrAddrOffset + 'b1; // update address since the current one has been sent
                            WrReqId = {{WRID_ZEROS{1'b0}}, 2'd2, rbbReqLineIdx_b[2*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:2*RBB_RD_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            rbbReqAck_b[2] = 'b1; // tell the RBB that the request has been sent to CCI
                        end
                        //if RBB is willing to write data but CCI is stalled, then do nothing
                        else if (rbbReqValid_b[2])
                        begin
                            //do nothing
                        end
                        //if RBB is not willing to write data, then we can infer that the RBB has sent all the write requests
                        //We can go to next RBB, and meanwhile claim that a result batch has been sent back
                        //Question: How to let CPU know it?
                        else
                        begin
                            if (rbb_pointer == (NUM_PEA-1))
                                rbb_pointer_d = 'b0;
                            else
                                rbb_pointer_d = rbb_pointer + 'b1;
                        end
                    end
                    if (rbbTestCmp_b[2]) begin
                        pearray_busy_d[2] = 1'b0;
                        status_updtd_d    = 1'b0;
                    end
                end
                'b11:
                begin
                    //If current rbb is empty, jump it without doing anything
                    if (rbbEmpty_b[3])
                    begin
                        if (rbb_pointer == (NUM_PEA-1))
                            rbb_pointer_d = 'b0;
                        else
                            rbb_pointer_d = rbb_pointer + 'b1;
                    end
                    //If current rbb is not empty, we are able to send data back to CCI
                    else
                    begin
                        //check if the RBB is willing to write data
                        if (rbbReqValid_b[3] && !ci2cf_C1TxAlmFull && status_updtd) //Not empty, Request Valid and CCI not stalled, then just write back data
                        begin
                            WrHdr_valid = 'b1; // a valid write request, 100% sent
                            WrAddrOffset_d = WrAddrOffset + 'b1; // update address since the current one has been sent
                            WrReqId = {{WRID_ZEROS{1'b0}}, 2'd3, rbbReqLineIdx_b[3*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:3*RBB_RD_ADDR_WIDTH]}; //Request ID is generated by combining the pointer and the line index
                            rbbReqAck_b[3] = 'b1; // tell the RBB that the request has been sent to CCI
                        end
                        //if RBB is willing to write data but CCI is stalled, then do nothing
                        else if (rbbReqValid_b[3])
                        begin
                            //do nothing
                        end
                        //if RBB is not willing to write data, then we can infer that the RBB has sent all the write requests
                        //We can go to next RBB, and meanwhile claim that a result batch has been sent back
                        //Question: How to let CPU know it?
                        else
                        begin
                            if (rbb_pointer == (NUM_PEA-1))
                                rbb_pointer_d = 'b0;
                            else
                                rbb_pointer_d = rbb_pointer + 'b1;
                        end
                    end
                    if (rbbTestCmp_b[3]) begin
                        pearray_busy_d[3] = 1'b0;
                        status_updtd_d    = 1'b0;
                    end
                end
            endcase
        end
    end

    //-------------------------
    // Handle CCI Tx Channels
    //-------------------------
    // Format Read Header
    wire [31:0]             RdAddr  = cr_src_address ^ {tbb_pointer, RdAddrOffset};
    wire [TXHDR_WIDTH-1:0]  RdHdr   = {
                                        5'h00,                          // [60:56]      Byte Enable
                                        rdreq_type,                     // [55:52]      Request Type
                                        6'h00,                          // [51:46]      Rsvd
                                        RdAddr,                         // [45:14]      Address
                                        RdReqId                         // [13:0]       Meta data to track the SPL requests
                                      };
    
        // Format Write Header
    wire [31:0]             WrAddr  = cr_dst_address ^ {rbb_pointer, WrAddrOffset};
    reg [DATA_WIDTH-1:0]   WrData;
    always @(*)
    begin
      WrData = 'b0;
            case(rbb_pointer)         /* synthesis parallel_case */
        'd0:  WrData = rbbRdDout_b[DATA_WIDTH-1+DATA_WIDTH*0:DATA_WIDTH*0];
        'd1:  WrData = rbbRdDout_b[DATA_WIDTH-1+DATA_WIDTH*1:DATA_WIDTH*1];
        'd2:  WrData = rbbRdDout_b[DATA_WIDTH-1+DATA_WIDTH*2:DATA_WIDTH*2];
        'd3:  WrData = rbbRdDout_b[DATA_WIDTH-1+DATA_WIDTH*3:DATA_WIDTH*3];
      endcase
    end
    wire [TXHDR_WIDTH-1:0]  WrHdr   = {
                                        5'h00,                          // [60:56]      Byte Enable
                                        wrreq_type,                     // [55:52]      Request Type
                                        6'h00,                          // [51:46]      Rsvd
                                        WrAddr,                         // [45:14]      Address
                                        WrReqId                         // [13:0]       Meta data to track the SPL requests
                                        };

  // Sending Requests
    always @(posedge clk)
    begin
        dsm_base_valid_q <= dsm_base_valid;

        if(!reset_n)
        begin
            afuid_updtd             <= 'b0;
            cf2ci_C1TxHdr           <= 'b0;
            cf2ci_C1TxWrValid       <= 'b0;
            cf2ci_C1TxData          <= 'b0;
            cf2ci_C0TxHdr           <= 'b0;
            cf2ci_C0TxRdValid       <= 'b0;
            dsm_base_valid_q        <= 'b0;
            status_updtd            <= 'b0;
        end
        else
        begin 
            //Tx Path
            //--------------------------------------------------------------------------
            cf2ci_C1TxHdr           <= 0;
            cf2ci_C1TxWrValid       <= 0;
            cf2ci_C0TxHdr           <= 0;
            cf2ci_C0TxRdValid       <= 0;
            status_updtd            <= status_updtd_d;


            // Channel 1
            if(ci2cf_C1TxAlmFull==0)
            begin
    //The first write request should be DSM initialization
                if( ci2cf_InitDn && dsm_base_valid_q && !afuid_updtd )
                begin
                    afuid_updtd             <= 1;
                    cf2ci_C1TxHdr           <= {
                                                    5'h0,                      // [60:56]      Byte Enable
                                                    WrLine,                    // [55:52]      Request Type
                                                    6'h00,                     // [51:46]      Rsvd
                                                    ds_afuid_address,          // [44:14]      Address
                                                    14'h3ffe                   // [13:0]       Meta data to track the SPL requests
                                               };                
                    cf2ci_C1TxWrValid       <= 1;
                    cf2ci_C1TxData          <= {    368'h0,                    // [512:144]    Zeros
                                                    VERSION ,                  // [143:128]    Version #2
                                                    BWA_MEM_SW                 // [127:0]      AFU ID
                                               };
                end
                else if (re2xy_go)  //Executing real tasks
                begin
                    if(status_updtd == 0) begin
                        status_updtd       <= 1'b1;
                        cf2ci_C1TxWrValid  <= 1'b1;
                        // cf2ci_C1TxHdr      <= {
                        //                             5'h0,                           // [60:56]      Byte Enable
                        //                             cr_wrthru_en? WrThru            // [55:52]      Req Type
                        //                                         : WrLine,           //
                        //                             6'h00,                          // [51:46]      Rsvd
                        //                             ds_stat_address,                // [44:14]      Address
                        //                             14'h3fff                        // [13:0]       Meta data to track the SPL requests
                        //                         };
                        // cf2ci_C1TxData     <= {     ab2re_ErrorInfo,                // [511:256] upper half cache line
                        //                             24'h00_0000,penalty_end,        // [255:224] test end overhead in # clks
                        //                             24'h00_0000,penalty_start,      // [223:192] test start overhead in # clks
                        //                             Num_Writes,                     // [191:160] Total number of Writes sent
                        //                             Num_Reads,                      // [159:128] Total number of Reads sent
                        //                             Num_ticks_high, Num_ticks_low,  // [127:64]  number of clks
                        //                             ErrorVector,                    // [63:32]   errors detected            
                        //                             32'h0000_0001                   // [31:0]    test completion flag
                        //                         };
                        cf2ci_C1TxHdr       <= {
                                                    5'h0,
                                                    WrLine,
                                                    6'h00,
                                                    ds_stat_address,
                                                    14'h3fff
                                                };
                        cf2ci_C1TxData      <= {
                                                    508'h0000,
                                                    pearray_busy                  // Write pearray status to DSM for software polling
                                                };
                    end else if( WrHdr_valid )                                          // Write to Destination Workspace
                    begin                                                               //-------------------------------------
                        cf2ci_C1TxHdr     <= WrHdr;
                        cf2ci_C1TxWrValid <= 1'b1;
                        cf2ci_C1TxData    <= WrData;
                    end
                end // re2xy_go
            end // C1_TxAmlFull

            // Channel 0
            if(  re2xy_go 
              && RdHdr_valid && !ci2cf_C0TxAlmFull )                                // Read from Source Workspace
            begin                                                                   //----------------------------------
                cf2ci_C0TxHdr      <= RdHdr;
                cf2ci_C0TxRdValid  <= 1;
            end

            /* synthesis translate_off */
            if(cf2ci_C1TxWrValid)
                $display("*Req Type: %x \t Addr: %x \n Data: %x", cf2ci_C1TxHdr[55:52], cf2ci_C1TxHdr[45:14], cf2ci_C1TxData);

            if(cf2ci_C0TxRdValid)
                $display("*Req Type: %x \t Addr: %x", cf2ci_C0TxHdr[55:52], cf2ci_C0TxHdr[45:14]);

            /* synthesis translate_on */

  end
    end

    //-------------------------
    //Handle Responses
    //-------------------------

    //We have already handled Cfg Responses in the Configuration Mode
    //We do not need to care about Write Responses
    //Only Read Responses are considered
    always @(*)
    begin
      tbbWrEn_b = 'b0;
      //tbbWrAddr_b = 'b0;
      //tbbWrDin_b = 'b0;
            case(rb2cf_C0RxHdr[13:TBB_WR_ADDR_WIDTH])         /* synthesis parallel_case */
        'd0:
          tbbWrEn_b[0] = re2xy_go && rb2cf_C0RxRdValid;
          //tbbWrDin_b[0*TBB_WR_DATA_WIDTH+TBB_WR_DATA_WIDTH-1:0*TBB_WR_DATA_WIDTH] = rb2cf_C0RxData;
          //tbbWrAddr_b[TBB_WR_ADDR_WIDTH*0+TBB_WR_ADDR_WIDTH-1:TBB_WR_ADDR_WIDTH*0] = rb2cf_C0RxHdr[TBB_WR_ADDR_WIDTH-1:0];
        'd1:
          tbbWrEn_b[1] = re2xy_go && rb2cf_C0RxRdValid;
          //tbbWrDin_b[1*TBB_WR_DATA_WIDTH+TBB_WR_DATA_WIDTH-1:1*TBB_WR_DATA_WIDTH] = rb2cf_C0RxData;
          //tbbWrAddr_b[TBB_WR_ADDR_WIDTH*1+TBB_WR_ADDR_WIDTH-1:TBB_WR_ADDR_WIDTH*1] = rb2cf_C0RxHdr[TBB_WR_ADDR_WIDTH-1:0];
        'd2:
          tbbWrEn_b[2] = re2xy_go && rb2cf_C0RxRdValid;
          //tbbWrDin_b[2*TBB_WR_DATA_WIDTH+TBB_WR_DATA_WIDTH-1:2*TBB_WR_DATA_WIDTH] = rb2cf_C0RxData;
          //tbbWrAddr_b[TBB_WR_ADDR_WIDTH*2+TBB_WR_ADDR_WIDTH-1:TBB_WR_ADDR_WIDTH*2] = rb2cf_C0RxHdr[TBB_WR_ADDR_WIDTH-1:0];
        'd3:
          tbbWrEn_b[3] = re2xy_go && rb2cf_C0RxRdValid;
          //tbbWrDin_b[3*TBB_WR_DATA_WIDTH+TBB_WR_DATA_WIDTH-1:3*TBB_WR_DATA_WIDTH] = rb2cf_C0RxData;
          //tbbWrAddr_b[TBB_WR_ADDR_WIDTH*3+TBB_WR_ADDR_WIDTH-1:TBB_WR_ADDR_WIDTH*3] = rb2cf_C0RxHdr[TBB_WR_ADDR_WIDTH-1:0];
      endcase
    end

    // Function: Returns physical address for a DSM register
    function automatic [31:0] dsm_offset2addr;
        input    [9:0]  offset_b;
        input    [63:0] base_b;
        begin
            dsm_offset2addr = base_b[37:6] + offset_b[9:6];
        end
    endfunction

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------

    generate
        genvar i;

        //-------------------------
        // TBB Generation
        //-------------------------

        for (i=0; i<NUM_PEA; i=i+1) begin: TBB
            tbb #(.TBB_WR_ADDR_WIDTH(TBB_WR_ADDR_WIDTH),
                  .TBB_WR_DATA_WIDTH(TBB_WR_DATA_WIDTH),
                  .TBB_RD_ADDR_WIDTH(TBB_RD_ADDR_WIDTH),
                  .TBB_RD_DATA_WIDTH(TBB_RD_DATA_WIDTH)
            )tbb
            (
                .clk            (clk),
                .reset_n        (reset_n),
                .task_start     (bm2pe_start_b[i]),
                .task_done      (rbbTestCmp_b[i]),
                .ReqValid       (tbbReqValid_b[i]),
                .ReqLineIdx     (tbbReqLineIdx_b[i*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:i*TBB_WR_ADDR_WIDTH]),
                .ReqAck         (tbbReqAck_b[i]),
                .WrEn           (tbbWrEn_b[i]),
                .WrAddr         (rb2cf_C0RxHdr[TBB_WR_ADDR_WIDTH-1:0]),
                //.WrAddr         (tbbWrAddr_b[i*TBB_WR_ADDR_WIDTH+TBB_WR_ADDR_WIDTH-1:i*TBB_WR_ADDR_WIDTH]),
                .WrDin          (rb2cf_C0RxData),
                //.WrDin          (tbbWrDin_b[i*TBB_WR_DATA_WIDTH+TBB_WR_DATA_WIDTH-1:i*TBB_WR_DATA_WIDTH]),
                .Full           (tbbFull_b[i]),
                .RdAddr         (tbbRdAddr_b[i*TBB_RD_ADDR_WIDTH+TBB_RD_ADDR_WIDTH-1:i*TBB_RD_ADDR_WIDTH]),
                .RdDout         (tbbRdDout_b[i*TBB_RD_DATA_WIDTH+TBB_RD_DATA_WIDTH-1:i*TBB_RD_DATA_WIDTH]),
                .Empty          (tbbEmpty_b[i])
            ); 
        end

        //-------------------------
        // RBB Generation
        //-------------------------

        for (i=0; i<NUM_PEA; i=i+1) begin: RBB
            rbb #(.RBB_WR_ADDR_WIDTH(RBB_WR_ADDR_WIDTH),
                  .RBB_WR_DATA_WIDTH(RBB_WR_DATA_WIDTH),
                  .RBB_RD_ADDR_WIDTH(RBB_RD_ADDR_WIDTH),
                  .RBB_RD_DATA_WIDTH(RBB_RD_DATA_WIDTH)
            )rbb
            (
                .clk            (clk),
                .reset_n        (reset_n),
                .task_done      (pe2bm_done_b[i]),
                .ReqValid       (rbbReqValid_b[i]),
                .ReqLineIdx     (rbbReqLineIdx_b[i*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:i*RBB_RD_ADDR_WIDTH]),
                .ReqAck         (rbbReqAck_b[i]),
                .WrEn           (rbbWrEn_b[i]),
                .WrAddr         (rbbWrAddr_b[i*RBB_WR_ADDR_WIDTH+RBB_WR_ADDR_WIDTH-1:i*RBB_WR_ADDR_WIDTH]),
                .WrDin          (rbbWrDin_b[i*RBB_WR_DATA_WIDTH+RBB_WR_DATA_WIDTH-1:i*RBB_WR_DATA_WIDTH]),
                .Full           (rbbFull_b[i]),
                .RdAddr         (rbbRdAddr_b[i*RBB_RD_ADDR_WIDTH+RBB_RD_ADDR_WIDTH-1:i*RBB_RD_ADDR_WIDTH]),
                .RdDout         (rbbRdDout_b[i*RBB_RD_DATA_WIDTH+RBB_RD_DATA_WIDTH-1:i*RBB_RD_DATA_WIDTH]),
                .Empty          (rbbEmpty_b[i]),
                .TestCmp        (rbbTestCmp_b[i])
            ); 
        end

        //-------------------------
        // PE_ARRAY Generation
        //-------------------------

        for (i = 0; i < NUM_PEA; i = i + 1) begin: PE_ARRAYS
            sw_pe_array sw_pe_array(
               .ap_clk            (clk),
               .ap_rst            (~reset_n),
               .ap_start          (bm2pe_start_b[i]),
               .ap_done           (pe2bm_done_b[i]),
               .ap_idle           (),
               .ap_ready          (),
               .ResData_we0       (rbbWrEn_b[i]),
               .ResData_ce0       (),
               .ResData_address0  (rbbWrAddr_b[i*RBB_WR_ADDR_WIDTH+RBB_WR_ADDR_WIDTH-1:i*RBB_WR_ADDR_WIDTH]),
               .ResData_q0        (),
               .ResData_d0        (rbbWrDin_b[i*RBB_WR_DATA_WIDTH+RBB_WR_DATA_WIDTH-1:i*RBB_WR_DATA_WIDTH]),
               .InData_we0        (),
               .InData_ce0        (),
               .InData_address0   (tbbRdAddr_b[i*TBB_RD_ADDR_WIDTH+TBB_RD_ADDR_WIDTH-1:i*TBB_RD_ADDR_WIDTH]),
               .InData_q0         (tbbRdDout_b[i*TBB_RD_DATA_WIDTH+TBB_RD_DATA_WIDTH-1:i*TBB_RD_DATA_WIDTH]),
               .InData_d0         ()
            ); 
        end
    endgenerate

endmodule
