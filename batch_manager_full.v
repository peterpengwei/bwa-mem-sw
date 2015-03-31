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

module batch_manager #(parameter TBB_ADDR_WIDTH, TBB_DATA_WIDTH, RBB_ADDR_WIDTH,
	RBB_DATA_WIDTH, NUM_PEA, TXHDR_WIDTH, RXHDR_WIDTH, DATA_WIDTH)
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
    //rb2cf_C0RxUMsgValid,              //                     cci_intf:           Rx UMsg valid
    //rb2cf_C0RxIntrValid,                //                     cci_intf:           Rx interrupt valid
    rb2cf_C1RxHdr,                    // [RXHDR_WIDTH-1:0]   cci_intf:           Rx header to SPL channel 1
    rb2cf_C1RxWrValid,                //                     cci_intf:           Rx write response valid
    //rb2cf_C1RxIntrValid,                //                     cci_intf:           Rx interrupt valid

    cf2ci_C0TxHdr,                    // [TXHDR_WIDTH-1:0]   cci_intf:           Tx Header from SPL channel 0
    cf2ci_C0TxRdValid,                //                     cci_intf:           Tx read request enable
    cf2ci_C1TxHdr,                    //                     cci_intf:           Tx Header from SPL channel 1
    cf2ci_C1TxData,                   //                     cci_intf:           Tx data from SPL
    cf2ci_C1TxWrValid,                //                     cci_intf:           Tx write request enable
    //cf2ci_C1TxIntrValid,              //                     cci_intf:           Tx interrupt valid
    ci2cf_C0TxAlmFull,                //                     cci_intf:           Tx memory channel 0 almost full
    ci2cf_C1TxAlmFull,                //                     cci_intf:           TX memory channel 1 almost full

    ci2cf_InitDn,                     // Link initialization is complete

    pe2bm_rbbWrEn_b,
    pe2bm_rbbWrAddr_b,
    pe2bm_rbbWrDin_b,
    bm2pe_rbbFull_b,

    pe2bm_tbbRdEn_b,
    pe2bm_tbbRdAddr_b,
    bm2pe_tbbRdDout_b,
    bm2pe_tbbEmpty_b
);

   input                        clk;                  //              in    std_logic;  -- Core clock
   input                        reset_n;              //              in    std_logic;  -- Use SPARINGLY only for control

   input [RXHDR_WIDTH-1:0]      rb2cf_C0RxHdr;        // [RXHDR_WIDTH-1:0]cci_intf:           Rx header to SPL channel 0
   input [DATA_WIDTH -1:0]      rb2cf_C0RxData;       // [DATA_WIDTH -1:0]cci_intf:           data response to SPL | no back pressure
   input                        rb2cf_C0RxWrValid;    //                  cci_intf:           write response enable
   input                        rb2cf_C0RxRdValid;    //                  cci_intf:           read response enable
   input                        rb2cf_C0RxCfgValid;   //                  cci_intf:           config response enable
   //input                        rb2cf_C0RxUMsgValid;  //                  cci_intf:           Rx UMsg valid
   //input                        rb2cf_C0RxIntrValid;    //                  cci_intf:           interrupt response enable
   input [RXHDR_WIDTH-1:0]      rb2cf_C1RxHdr;        // [RXHDR_WIDTH-1:0]cci_intf:           Rx header to SPL channel 1
   input                        rb2cf_C1RxWrValid;    //                  cci_intf:           write response valid
   //input                        rb2cf_C1RxIntrValid;    //                  cci_intf:           interrupt response valid

   output [TXHDR_WIDTH-1:0]     cf2ci_C0TxHdr;        // [TXHDR_WIDTH-1:0]cci_intf:           Tx Header from SPL channel 0
   output                       cf2ci_C0TxRdValid;    //                  cci_intf:           Tx read request enable
   output [TXHDR_WIDTH-1:0]     cf2ci_C1TxHdr;        //                  cci_intf:           Tx Header from SPL channel 1
   output [DATA_WIDTH -1:0]     cf2ci_C1TxData;       //                  cci_intf:           Tx data from SPL
   output                       cf2ci_C1TxWrValid;    //                  cci_intf:           Tx write request enable
   //output                       cf2ci_C1TxIntrValid;  //                  cci_intf:           Tx interrupt valid
   input                        ci2cf_C0TxAlmFull;    //                  cci_intf:           Tx memory channel 0 almost full
   input                        ci2cf_C1TxAlmFull;    //                  cci_intf:           TX memory channel 1 almost full
   
   input                        ci2cf_InitDn;         //                  cci_intf:           Link initialization is complete

   input [NUM_PEA-1:0]			pe2bm_rbbWrEn_b;
   input [RBB_ADDR_WIDTH*NUM_PEA-1:0]	pe2bm_rbbWrAddr_b;
   input [RBB_DATA_WIDTH*NUM_PEA-1:0]	pe2bm_rbbWrDin_b;
   output [NUM_PEA-1:0]			bm2pe_rbbFull_b;

   input [NUM_PEA-1:0]			pe2bm_tbbRdEn_b;
   input [TBB_ADDR_WIDTH*NUM_PEA-1:0]	pe2bm_tbbRdAddr_b;
   output [TBB_DATA_WIDTH*NUM_PEA-1:0]	bm2pe_tbbRdDout_b;
   output [NUM_PEA-1:0]			bm2pe_tbbEmpty_b;

////   localparam      PEND_THRESH = 7;
////   localparam      ADDR_LMT    = 20;
////   localparam      MDATA       = 'd11;
   
////   wire                         re2xy_go;
////   wire [31:0]                  re2xy_src_addr;
////   wire [31:0]                  re2xy_dst_addr;
////   wire [31:0]                  re2xy_NumLines;
////   wire                         re2xy_Cont;
////   wire [7:0]                   re2xy_test_cfg;
////   wire [2:0]                   re2ab_Mode;
////   wire                         ab2re_TestCmp;
////   wire [255:0]                 ab2re_ErrorInfo;
////   wire                         ab2re_ErrorValid;
////   
////   wire                         test_reset_n;

    //----------------------------------------------------------------------------------------------------------------------
    // NLB v1.1 AFU ID
    localparam       BWA_MEM_SW          = 128'h2015_0212_900d_beef_0000_0000_0000_0000;
    localparam       VERSION             = 16'h0001;
    localparam       CONST_FWDRANGE_BEG  = 16'h1B00;                 //      fwd all CSR writes above this address to test modules
    
    //---------------------------------------------------------
    // CCI-S Request Encodings  ***** DO NOT MODIFY ******
    //---------------------------------------------------------
    localparam       WrThru              = 4'h1;
    localparam       WrLine              = 4'h2;
    //localparam       RdLine_S            = 4'h4;
    localparam       RdLine              = 4'h4;
    localparam       WrFence             = 4'h5;
    //localparam       RdLine_I            = 4'h6;
    //localparam       RdLine_O            = 4'h7;
    //localparam       Intr                = 4'h8;    // FPGA to CPU interrupt
    
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

    //? Why not aligned?
    localparam      DEF_DSM_BASE         = 32'h04ff_ffff;           // default status address
    
    //---------------------------------------------------------
    // CSR Address Map ***** DO NOT MODIFY *****
    //---------------------------------------------------------
    localparam      CSR_AFU_DSM_BASEL    = 16'h1a00;                 // WO - Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
    localparam      CSR_AFU_DSM_BASEH    = 16'h1a04;                 // WO - Upper 32-bits of AFU DSM base address.
    localparam      CSR_SRC_ADDR         = 16'h1a20;                 // WO   Reads are targetted to this region 
    localparam      CSR_DST_ADDR         = 16'h1a24;                 // WO   Writes are targetted to this region
    localparam      CSR_NUM_BATCHES        = 16'h1a28;                 // WO   Numbers of cache lines to be read/write
    //localparam      CSR_CTL              = 16'h1a2c;                 // WO   Control CSR to start n stop the test
    //localparam      CSR_CFG              = 16'h1a34;                 // WO   Configures test mode, wrthru, cont and delay mode
    //localparam      CSR_INACT_THRESH     = 16'h1a38;                 // WO   set the threshold limit for inactivity trigger
    //localparam      CSR_INTERRUPT0       = 16'h1a3c;                 // WO   SW allocates Interrupt APIC ID & Vector
    
    //----------------------------------------------------------------------------------
    // Device Status Memory (DSM) Address Map ***** DO NOT MODIFY *****
    // Physical address = value at CSR_AFU_DSM_BASE + Byte offset
    //----------------------------------------------------------------------------------
    //                                     Byte Offset                 Attribute    Width   Comments
    localparam      DSM_AFU_ID           = 32'h0;                   // RO           32b     non-zero value to uniquely identify the AFU
    localparam      DSM_STATUS           = 32'h40;                  // RO           512b    test status and error info
    
    //----------------------------------------------------------------------------------------------------------------------
    
    reg  [DATA_WIDTH-1:0]   cf2ci_C1TxData;
    reg  [TXHDR_WIDTH-1:0]  cf2ci_C1TxHdr;
    reg                     cf2ci_C1TxWrValid;
    reg  [TXHDR_WIDTH-1:0]  cf2ci_C0TxHdr;
    reg                     cf2ci_C0TxRdValid;
    //reg                     cf2ci_C1TxIntrValid;
    
    //reg  [31:0]             ErrorVector;
    //reg  [31:0]             Num_Reads;                              // Number of reads performed
    //reg  [31:0]             Num_Writes;                             // Number of writes performed
    //reg  [31:0]             Num_ticks_low, Num_ticks_high;
    //reg  [PEND_THRESH-1:0]  Num_Pend;                               // Number of pending requests
    //reg  [31:0]             Num_C0stall;                            // Number of clocks for which Channel0 was throttled
    //reg  [31:0]             Num_C1stall;                            // Number of clocks for which channel1 was throttled
    //reg  signed [31:0]      Num_RdCredits;                          // For LPBK1: number of read credits
    //reg                     re2ab_stallRd;
    //reg                     RdHdr_valid;
    //wire                    WrHdr_valid;
    //reg  [31:0]             wrfifo_addr;
    //reg  [DATA_WIDTH-1:0]   wrfifo_data;
    //reg                     txFifo_RdAck;
    //reg  [DATA_WIDTH-1:0]   rb2cf_C0RxData_q;
    //reg  [RXHDR_WIDTH-1:0]  rb2cf_C0RxHdr_q;
    //reg                     rb2cf_C0RxWrValid_q;
    //reg                     rb2cf_C0RxRdValid_q;
    //reg                     rb2cf_C0RxUMsgValid_q;
    //reg                     re2ab_CfgValid_d;
    //reg                     re2ab_RdSent;
    //reg                     status_write;
    //reg                     interrupt_sent;
    //reg                     send_interrupt;
    
    //reg   [31:0]            inact_cnt;
    //reg                     inact_timeout;
    //reg   [5:0]             delay_lfsr;
    //reg   [31:0]            cr_inact_thresh;
    //reg                     penalty_start_f;
    //reg   [7:0]             penalty_start;
    //reg   [7:0]             penalty_end;
    reg                     dsm_base_valid;
    reg                     dsm_base_valid_q;
    reg                     afuid_updtd;
    //reg   [3:0]             rdreq_type;
    //reg   [3:0]             rnd_rdreq_type;
    //reg   [1:0]             rnd_rdreq_sel;
    
    integer                 i;
    reg   [63:0]            cr_dsm_base;                            // a00h, a04h - DSM base address
    reg   [31:0]            cr_src_address;                         // a20h - source buffer address
    reg   [31:0]            cr_dst_address;                         // a24h - destn buffer address
    reg   [31:0]            cr_num_batches;                         // a28h - Number of batches available for processing
    reg   [31:0]            cr_ctl = 0;                             // a2ch - control register to start and stop the test
    //reg                     cr_wrthru_en;                           // a34h - [0]    : test configuration- wrthru_en
    //reg                     cr_cont;                                // a34h - [1]    : repeats the test sequence, NO end condition
    //reg   [2:0]             cr_mode;                                // a34h - [4:2]  : selects test mode
    //reg                     cr_delay_en;                            // a34h - [8]    : use start delay
    //reg   [1:0]             cr_rdsel, cr_rdsel_q;                   // a34h - [10:9] : read request type
    //reg   [7:0]             cr_test_cfg;                            // a34h - [27:0] : configuration within a selected test mode
    //reg   [31:0]            cr_interrupt0;                          // a3ch - SW allocates apic id & interrupt vector
    //reg                     cr_interrupt_testmode;
    //reg                     cr_interrupt_on_error;
    reg   [31:0]            ds_afuid_address;                        // 0h - afu id is written to this address
    reg   [31:0]            ds_stat_address;                        // 40h - test status is written to this address
    //wire  [31:0]            re2xy_src_addr  = cr_src_address;
    //wire  [31:0]            re2xy_dst_addr  = cr_dst_address;
    
    //wire                    txFifo_Full;
    //wire                    txFifo_AlmFull;
    //wire [13:0]             rxfifo_Din      = rb2cf_C1RxHdr[13:0];
    //wire                    rxfifo_WrEn     = rb2cf_C1RxWrValid;
    //wire                    rxfifo_Full;
    //
    //wire [13:0]             rxfifo_Dout;
    //wire                    rxfifo_Dout_v;
    wire                    test_reset_n     = cr_ctl[0];                // Clears all the states. Either is one then test is out of Reset.
    wire                    test_go         = cr_ctl[1];                // When 0, it allows reconfiguration of test parameters.
    //wire [2:0]              re2ab_Mode      = cr_mode;
    //wire                    re2ab_WrSent    = !txFifo_Full;             // stop accepting new requests, after status write=1
    //wire                    txFifo_WrEn     = (ab2re_WrEn| ab2re_WrFence) && ~txFifo_Full;
    //wire [13:0]             txFifo_WrReqId;
    //wire [ADDR_LMT-1:0]     txFifo_WrAddr;
    //wire                    txFifo_WrFence;

    //CCI Read Address Offset
    reg [TBB_ADDR_WIDTH-1:0]	RdAddrOffset;
    //CCI Read ID
    reg [13:0]			RdReqId;
    //CCI Read Type
    wire [3:0]			rdreq_type = RdLine;
    
        // Format Read Header
    wire [31:0]             RdAddr  = cr_src_address ^ RdAddrOffset;
    wire [TXHDR_WIDTH-1:0]  RdHdr   = {
                                        5'h00,                          // [60:56]      Byte Enable
                                        rdreq_type,                     // [55:52]      Request Type
                                        6'h00,                          // [51:46]      Rsvd
                                        RdAddr,                         // [45:14]      Address
                                        RdReqId                         // [13:0]       Meta data to track the SPL requests
                                      };

    //CCI Write Address Offset
    reg [RBB_ADDR_WIDTH-1:0]	WrAddrOffset;
    //CCI Write ID
    reg [13:0]			WrReqId;
    //CCI Write Type
    wire [3:0]			wrreq_type = WrLine;
    
        // Format Write Header
    wire [31:0]             WrAddr      = cr_dst_address ^ WrAddrOffset;
    wire [DATA_WIDTH-1:0]   WrData;
    wire [TXHDR_WIDTH-1:0]  WrHdr   = {
                                        5'h00,                          // [60:56]      Byte Enable
                                        wrreq_type,                     // [55:52]      Request Type
                                        6'h00,                          // [51:46]      Rsvd
                                        WrAddr,                         // [45:14]      Address
                                        WrReqId                         // [13:0]       Meta data to track the SPL requests
                                        };


    //wire                    re2ab_RdRspValid = rb2cf_C0RxRdValid_q;
    //wire                    re2ab_UMsgValid  = rb2cf_C0RxUMsgValid_q;
    //wire                    re2ab_CfgValid   = re2ab_CfgValid_d;
    //wire   [13:0]           re2ab_RdRsp      = rb2cf_C0RxHdr_q[13:0];
    //wire   [DATA_WIDTH-1:0] re2ab_RdData     = rb2cf_C0RxData_q;
    //wire                    re2ab_WrRspValid = rxfifo_Dout_v | rb2cf_C0RxWrValid_q;
    //wire   [13:0]           re2ab_WrRsp      = rb2cf_C0RxWrValid_q ? rb2cf_C0RxHdr_q[13:0] : rxfifo_Dout;
    reg                     re2xy_go;
    //wire   [31:0]           re2xy_NumLines   = cr_num_lines;
    //wire                    re2xy_Cont       = cr_cont;
    //wire                    re2ab_WrAlmFull  = txFifo_AlmFull;
    //wire                    rnd_delay        = ~cr_delay_en || (delay_lfsr[0] || delay_lfsr[2] || delay_lfsr[3]);
    //wire   [7:0]            re2xy_test_cfg   = cr_test_cfg;
    //wire                    tx_errorValid    = ErrorVector!=0;


    always @(posedge clk)                                              // - Update Test Configuration
    begin                                                                   //-----------------------------
        if(!reset_n)
        begin
            cr_dsm_base     <= DEF_DSM_BASE;
            cr_src_address  <= DEF_SRC_ADDR;
            cr_dst_address  <= DEF_DST_ADDR;
	    cr_num_batches  <= 0;
            cr_ctl          <= 0;
            //cr_wrthru_en    <= 0;
            //cr_mode         <= 0;
            //cr_cont         <= 0;
            //cr_num_lines    <= 8;
            //cr_delay_en     <= 0;
            //cr_test_cfg     <= 0;
            //cr_inact_thresh <= 32'hffff_ffff;
            //cr_interrupt0   <= 0;
            //cr_interrupt_on_error <= 0;
            //cr_interrupt_testmode <= 0;
            dsm_base_valid  <= 0;
        end
        else
        begin                  
            if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
                    CSR_CTL          :   cr_ctl             <= rb2cf_C0RxData[31:0];
                    CSR_AFU_DSM_BASEH:   cr_dsm_base[63:32] <= rb2cf_C0RxData[31:0];
                    CSR_AFU_DSM_BASEL:begin
                                         cr_dsm_base[31:0]  <= rb2cf_C0RxData[31:0];
                                         dsm_base_valid     <= 1;
                                      end
		    //cr_num_batches corresponds to the number of task batches available for processing
                    CSR_NUM_BATCHES:     cr_num_batches     <= cr_num_batches + 1;
                endcase

            if(test_reset_n && ~test_go) // Configuration Mode, following CSRs can only be updated in this mode
            begin
                if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
                    CSR_SRC_ADDR:        cr_src_address     <= rb2cf_C0RxData[31:0];
                    CSR_DST_ADDR:        cr_dst_address     <= rb2cf_C0RxData[31:0];
                    //CSR_INACT_THRESH:    cr_inact_thresh    <= rb2cf_C0RxData[31:0];
                    //CSR_INTERRUPT0:      cr_interrupt0      <= rb2cf_C0RxData[31:0];
                    //CSR_CFG:        begin
                    //                     cr_wrthru_en       <= rb2cf_C0RxData[0];
                    //                     cr_cont            <= rb2cf_C0RxData[1];
                    //                     cr_mode            <= rb2cf_C0RxData[4:2];
                    //                     cr_delay_en        <= rb2cf_C0RxData[8];
                    //                     cr_rdsel           <= rb2cf_C0RxData[10:9];
                    //                     cr_test_cfg        <= rb2cf_C0RxData[27:20];
                    //                     cr_interrupt_on_error <= rb2cf_C0RxData[28];
                    //                     cr_interrupt_testmode <= rb2cf_C0RxData[29];
                    //                 end
                endcase
            end
        end
    end

    always @(posedge clk)
    begin
        ds_stat_address  <= dsm_offset2addr(DSM_STATUS,cr_dsm_base);
        ds_afuid_address <= dsm_offset2addr(DSM_AFU_ID,cr_dsm_base);
        dsm_base_valid_q <= dsm_base_valid;
        //cr_rdsel_q       <= cr_rdsel;
        //delay_lfsr <= {delay_lfsr[4:0], (delay_lfsr[5] ^ delay_lfsr[4]) };

        //case(cr_rdsel_q)
        //    2'h0:   rdreq_type <= RdLine_S;
        //    2'h1:   rdreq_type <= RdLine_I;
        //    2'h2:   rdreq_type <= RdLine_O;
        //    2'h3:   rdreq_type <= rnd_rdreq_type;
        //endcase
        //rnd_rdreq_sel  <= delay_lfsr%3;
        //case(rnd_rdreq_sel)
        //    2'h1:   rnd_rdreq_type <= RdLine_I;
        //    2'h2:   rnd_rdreq_type <= RdLine_O;
        //    default:rnd_rdreq_type <= RdLine_S;
        //endcase

        if(test_go & ci2cf_InitDn  & afuid_updtd)                                             
            re2xy_go    <= 1;
        //if(status_write)
        //    re2xy_go    <= 0;
        //
        //send_interrupt <= (cr_interrupt_on_error & tx_errorValid) | cr_interrupt_testmode;
        //Tx Path
        //--------------------------------------------------------------------------
        cf2ci_C1TxHdr           <= 0;
        cf2ci_C1TxWrValid       <= 0;
        //cf2ci_C1TxIntrValid     <= 0;
        cf2ci_C0TxHdr           <= 0;
        cf2ci_C0TxRdValid       <= 0;

        // Channel 1
        if(ci2cf_C1TxAlmFull==0)
        begin
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
            //else if ( status_write
            //        & send_interrupt
            //        & !interrupt_sent
            //        )
            //begin
            //    interrupt_sent          <= 1;
            //    cf2ci_C1TxHdr           <= {
            //                                    5'h0,                      // [60:56]      Byte Enable
            //                                    Intr,                      // [55:52]      Request Type
            //                                    6'h00,                     // [51:46]      Rsvd
            //                                    cr_interrupt0,             // [44:14]      Address
            //                                    14'h3ffc                   // [13:0]       Meta data to track the SPL requests
            //                                };                
            //    cf2ci_C1TxIntrValid     <= 1;
            //end
            else if (re2xy_go)
            //else if (re2xy_go & rnd_delay)
            begin
                //if( ab2re_TestCmp                                               // Update Status upon test completion
                //    ||tx_errorValid                                             // Error detected 
                //    ||cr_ctl[2]                                                 // SW forced test termination
                //  )                       
                //begin                                                           //-----------------------------------
                //    status_write       <= 1'b1;
                //    if(status_write==0)
                //        cf2ci_C1TxWrValid  <= 1'b1;
                //    cf2ci_C1TxHdr      <= {
                //                                5'h0,                           // [60:56]      Byte Enable
                //                                cr_wrthru_en? WrThru            // [55:52]      Req Type
                //                                            : WrLine,           //
                //                                6'h00,                          // [51:46]      Rsvd
                //                                ds_stat_address,                // [44:14]      Address
                //                                14'h3fff                        // [13:0]       Meta data to track the SPL requests
                //                            };
                //    cf2ci_C1TxData     <= {     ab2re_ErrorInfo,               // [511:256] upper half cache line
                //                                24'h00_0000,penalty_end,       // [255:224] test end overhead in # clks
                //                                24'h00_0000,penalty_start,     // [223:192] test start overhead in # clks
                //                                Num_Writes,                    // [191:160] Total number of Writes sent
                //                                Num_Reads,                     // [159:128] Total number of Reads sent
                //                                Num_ticks_high, Num_ticks_low, // [127:64]  number of clks
                //                                ErrorVector,                   // [63:32]   errors detected            
                //                                32'h0000_0001                  // [31:0]    test completion flag
                //                            };
                //end
                //else if( WrHdr_valid )                                         // Write to Destination Workspace
                if( WrHdr_valid )                                         // Write to Destination Workspace
                begin                                                          //-------------------------------------
                    cf2ci_C1TxHdr     <= WrHdr;
                    cf2ci_C1TxWrValid <= 1'b1;
                    cf2ci_C1TxData    <= WrData;
                    //Num_Writes        <= Num_Writes + 1'b1;
                end
            end // re2xy_go
        end // C1_TxAmlFull

        // Channel 0
        //if(  re2xy_go && rnd_delay 
        if(  re2xy_go 
          && RdHdr_valid && !ci2cf_C0TxAlmFull )                                // Read from Source Workspace
        begin                                                                   //----------------------------------
            cf2ci_C0TxHdr      <= RdHdr;
            cf2ci_C0TxRdValid  <= 1;
            //Num_Reads          <= Num_Reads + 1'b1;
        end

        //--------------------------------------------------------------------------
        //Rx Response Path
        //--------------------------------------------------------------------------
        rb2cf_C0RxData_q       <= rb2cf_C0RxData;
        rb2cf_C0RxHdr_q        <= rb2cf_C0RxHdr;
        rb2cf_C0RxWrValid_q    <= rb2cf_C0RxWrValid;
        rb2cf_C0RxRdValid_q    <= rb2cf_C0RxRdValid;
        rb2cf_C0RxUMsgValid_q  <= rb2cf_C0RxUMsgValid;
        re2ab_CfgValid_d       <= rb2cf_C0RxCfgValid && (rb2cf_C0RxHdr[9-:4]>=(CONST_FWDRANGE_BEG>>8));

        //// Counters
        ////--------------------------------------------------------------------------

        //if(re2xy_go)                                                // Count #clks after test start
        //begin
        //    Num_ticks_low   <= Num_ticks_low + 1'b1;
        //    if(&Num_ticks_low)
        //        Num_ticks_high  <= Num_ticks_high + 1'b1;
        //end

        //if(re2xy_go & ci2cf_C0TxAlmFull)
        //    Num_C0stall     <= Num_C0stall + 1'b1;

        //if(re2xy_go & ci2cf_C1TxAlmFull)
        //    Num_C1stall     <= Num_C1stall + 1'b1;

        //case({cf2ci_C1TxWrValid, cf2ci_C0TxRdValid, rb2cf_C1RxWrValid,(rb2cf_C0RxRdValid|rb2cf_C0RxWrValid)})
        //    4'b0001:    Num_Pend    <= Num_Pend - 2'h1;
        //    4'b0010:    Num_Pend    <= Num_Pend - 2'h1;
        //    4'b0011:    Num_Pend    <= Num_Pend - 2'h2;
        //    4'b0100:    Num_Pend    <= Num_Pend + 2'h1;
        //    //4'b0101:    
        //    //4'b0110:
        //    4'b0111:    Num_Pend    <= Num_Pend - 2'h1;
        //    4'b1000:    Num_Pend    <= Num_Pend + 2'h1;
        //    //4'b1001:    
        //    //4'b1010:
        //    4'b1011:    Num_Pend    <= Num_Pend - 2'h1;
        //    4'b1100:    Num_Pend    <= Num_Pend + 2'h2;
        //    4'b1101:    Num_Pend    <= Num_Pend + 2'h1;
        //    4'b1110:    Num_Pend    <= Num_Pend + 2'h1;
        //    //4'b1111:
        //endcase                

        //// For LPBK1 (memory copy): stall reads  if Num_RdCredits less than 0. Read credits are limited by the depth of Write fifo
        //// Wr fifo depth in requestor is 128. Therefore max num write pending should be less than 128.
        //if(cf2ci_C0TxRdValid && !cf2ci_C1TxWrValid)
        //    Num_RdCredits <= Num_RdCredits - 1'b1;

        //if(!cf2ci_C0TxRdValid && cf2ci_C1TxWrValid )
        //    Num_RdCredits <= Num_RdCredits + 1'b1;

        //re2ab_stallRd     <= ($signed(Num_RdCredits)<=0);
        //
        //// Error Detection Logic
        ////--------------------------
        //if(Num_Pend<0)
        //begin
        //    ErrorVector[0]  <= 1;
        //    /*synthesis translate_off */
        //    $display("nlb_lpbk: Error: unexpected Rx response");
        //    /*synthesis translate_on */
        //end

        //if(rxfifo_Full & rxfifo_WrEn)
        //begin
        //    ErrorVector[1]  <= 1;
        //    /*synthesis translate_off */
        //    $display("nlb_lpbk: Error: WrRx fifo overflow");
        //    /*synthesis translate_on */
        //end

        //if(txFifo_Full & txFifo_WrEn)
        //begin
        //    ErrorVector[2]  <= 1;
        //    /*synthesis translate_off */
        //    $display("nlb_lpbk: Error: wr fifo overflow");
        //    /*synthesis translate_on */
        //end

        //if(ErrorVector[3]==0)
        //    ErrorVector[3]  <= ab2re_ErrorValid;

        /* synthesis translate_off */
        if(cf2ci_C1TxWrValid)
            $display("*Req Type: %x \t Addr: %x \n Data: %x", cf2ci_C1TxHdr[55:52], cf2ci_C1TxHdr[45:14], cf2ci_C1TxData);

        if(cf2ci_C0TxRdValid)
            $display("*Req Type: %x \t Addr: %x", cf2ci_C0TxHdr[55:52], cf2ci_C0TxHdr[45:14]);

        /* synthesis translate_on */


        // Use for Debug- if no transactions going across the CCI interface # clks > inactivity threshold 
        // than set the flag. You may use this as a trigger signal in logic analyzer
        //if(cf2ci_C1TxWrValid || cf2ci_C0TxRdValid)
        //    inact_cnt  <= 0;
        //else if(re2xy_go)
        //    inact_cnt  <= inact_cnt + 1;

        //if(inact_timeout==0)
        //begin
        //    if(inact_cnt>=cr_inact_thresh)
        //        inact_timeout   <= 1;
        //end
        //else if(cf2ci_C1TxWrValid || cf2ci_C0TxRdValid)
        //begin
        //    inact_timeout   <= 0;
        //end

        if(!test_reset_n)
        begin
            //Num_Reads               <= 0;
            //Num_Writes              <= 0;
            //Num_Pend                <= 0;
            //Num_ticks_low           <= 0;
            //Num_ticks_high          <= 0;
            re2xy_go                <= 0;
            //rb2cf_C0RxData_q        <= 0;
            //rb2cf_C0RxHdr_q         <= 0;
            //rb2cf_C0RxWrValid_q     <= 0;
            //rb2cf_C0RxRdValid_q     <= 0;
            //rb2cf_C0RxUMsgValid_q   <= 0;
            //re2ab_CfgValid_d        <= 0;
            //ErrorVector             <= 0;
            //status_write            <= 0;
            //interrupt_sent          <= 0;
            //send_interrupt          <= 0;
            //inact_cnt               <= 0;
            //inact_timeout           <= 0;
            //delay_lfsr              <= 1;
            //Num_C0stall             <= 0;
            //Num_C1stall             <= 0;
            //Num_RdCredits           <= (2**PEND_THRESH-8);
        end
        if(!reset_n)
        begin
            afuid_updtd             <= 0;
            dsm_base_valid_q        <= 0;
        end
    end

    //always @(posedge clk)                                                      // Computes NLB start and end overheads
    //begin                                                                           //-------------------------------------
    //    if(!test_go)
    //    begin
    //        penalty_start   <= 0;
    //        penalty_start_f <= 0;
    //        penalty_end     <= 2;
    //    end
    //    else
    //    begin
    //        if(!penalty_start_f & (cf2ci_C0TxRdValid | cf2ci_C1TxWrValid ))
    //        begin
    //            penalty_start_f   <= 1;
    //            penalty_start     <= Num_ticks_low[7:0];                    /* synthesis translate_off */
    //            $display ("NLB_INFO : start penalty = %d ", Num_ticks_low); /* synthesis translate_on */
    //        end

    //        penalty_end <= penalty_end + 1'b1;
    //        if(rb2cf_C0RxWrValid | rb2cf_C0RxRdValid | rb2cf_C0RxUMsgValid 
    //        | rb2cf_C1RxWrValid )
    //        begin
    //            penalty_end     <= 2;
    //        end

    //        if(ab2re_TestCmp
    //          && !ci2cf_C1TxAlmFull
    //          && !status_write)
    //        begin                                                       /* synthesis translate_off */
    //            $display ("NLB_INFO : end penalty = %d ", penalty_end); /* synthesis translate_on */
    //        end

    //    end
    //end

    //always @(*)
    //begin
    //    RdHdr_valid = re2xy_go
    //    && !status_write
    //    && rnd_delay
    //    && !ci2cf_C0TxAlmFull
    //    && ab2re_RdEn;

    //    re2ab_RdSent= RdHdr_valid;

    //    txFifo_RdAck = re2xy_go && rnd_delay  && !ci2cf_C1TxAlmFull && WrHdr_valid;

    //end

    ////----------------------------------------------------------------------------------------------------------------------------------------------
    ////                                                              Instances
    ////----------------------------------------------------------------------------------------------------------------------------------------------
    //// Tx Write request fifo. Some tests may have writes dependent on reads, i.e. a read response will generate a write request
    //// If the CCI-S write channel is stalled, then the write requests will be queued up in this Tx fifo.

    //wire [1+512+ADDR_LMT+13:0]txFifo_Din    = {ab2re_WrFence,
    //                                           ab2re_WrDin,
    //                                           ab2re_WrAddr, 
    //                                           ab2re_WrTID
    //                                          };
    //wire [1+512+ADDR_LMT+13:0]txFifo_Dout;
    //assign                  txFifo_WrAddr   = txFifo_Dout[ADDR_LMT-1+14:14];
    //assign                  WrData          = txFifo_Dout[511+ADDR_LMT+14:ADDR_LMT+14];
    //assign                  txFifo_WrFence  = txFifo_Dout[1+512+ADDR_LMT+13];
    //assign                  txFifo_WrReqId  = txFifo_Dout[13:0];
    //nlb_sbv_gfifo  #(.DATA_WIDTH  (1+DATA_WIDTH+ADDR_LMT+14),
    //                 .DEPTH_BASE2 (PEND_THRESH),
    //                 .FULL_THRESH (2**PEND_THRESH-3)  
    //)nlb_writeTx_fifo
    //(                                       //--------------------- Input  ------------------
    //    .reset_n         (test_reset_n),
    //    .Clk            (clk),    
    //    .fifo_din       (txFifo_Din),          
    //    .fifo_wen       (txFifo_WrEn),      
    //    .fifo_rdack     (txFifo_RdAck),
    //                                       //--------------------- Output  ------------------
    //    .fifo_dout      (txFifo_Dout),        
    //    .fifo_dout_v    (WrHdr_valid),
    //    .fifo_empty     (),
    //    .fifo_full      (txFifo_Full),
    //    .fifo_count     (),
    //    .fifo_almFull   (txFifo_AlmFull)
    //); 

    //wire    rxfifo_RdAck    = rxfifo_Dout_v & ~rb2cf_C0RxWrValid_q;
    //
    //// CCI-S could return two write responses per clock, but arbiter can accept only 1 write response per clock. 
    //// This fifo will store the second write response
    //nlb_sbv_gfifo  #(.DATA_WIDTH  ('d14),
    //                .DEPTH_BASE2 (PEND_THRESH)
    //)nlb_writeRx_fifo  
    //(                                      //--------------------- Input  ------------------
    //    .reset_n         (test_reset_n),
    //    .Clk            (clk),
    //    .fifo_din       (rxfifo_Din),          
    //    .fifo_wen       (rxfifo_WrEn),      
    //    .fifo_rdack     (rxfifo_RdAck),        
    //                                       //--------------------- Output  ------------------
    //    .fifo_dout      (rxfifo_Dout),
    //    .fifo_dout_v    (rxfifo_Dout_v),
    //    .fifo_empty     (),
    //    .fifo_full      (rxfifo_Full),
    //    .fifo_count     (),
    //    .fifo_almFull   ()
    //);


    // Function: Returns physical address for a DSM register
    function automatic [31:0] dsm_offset2addr;
        input    [9:0]  offset_b;
        input    [63:0] base_b;
        begin
            dsm_offset2addr = base_b[37:6] + offset_b[9:6];
        end
    endfunction

generate
  genvar i;
  for (i=0; i<NUM_PEA; i=i+1) begin
	  pe_array pe_array(
	     .clk					(clk),
	     .reset_n					(reset_n),
	     .pe2bm_rbbWrEn 	         		(pe2bm_rbbWrEn_b[i]),
    	     .pe2bm_rbbWrAddr				(pe2bm_rbbWrAddr_b[i*RBB_ADDR_WIDTH+RBB_ADDR_WIDTH-1:i*RBB_ADDR_WIDTH]),
    	     .pe2bm_rbbWrDin				(pe2bm_rbbWrDin_b[i*RBB_DATA_WIDTH+RBB_DATA_WIDTH-1:i*RBB_DATA_WIDTH]),
    	     .bm2pe_rbbFull 				(bm2pe_rbbFull_b[i]),
    	     .pe2bm_tbbRdEn 				(pe2bm_tbbRdEn_b[i]),
    	     .pe2bm_tbbRdAddr				(pe2bm_tbbRdAddr_b[i*TBB_ADDR_WIDTH+TBB_ADDR_WIDTH-1:i*TBB_ADDR_WIDTH]),
    	     .bm2pe_tbbRdDout				(bm2pe_tbbRdDout_b[i*TBB_DATA_WIDTH+TBB_DATA_WIDTH-1:i*TBB_DATA_WIDTH]),
    	     .bm2pe_tbbEmpty 				(bm2pe_tbbEmpty_b[i])
	  ); 
  end
endgenerate

endmodule

