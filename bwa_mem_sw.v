// ***************************************************************************
//
//        UCLA CDSC BWA-MEM Smith-Waterman AFU
//
// Engineer:            Peng Wei
// Create Date:         Feb 11, 2015
// Module Name:         bwa_mem_sw
// Description:         top level wrapper for BWA-MEM S-W AFU
//                      it instantiates Batch Buffer and PE Arrays
// ***************************************************************************
//
// CSR Address Map -- Change v1.1
//------------------------------------------------------------------------------------------
//      Address[15:0] Attribute         Name                    Comments
//     'h1A00          WO                CSR_AFU_DSM_BASEL       Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
//     'h1A04          WO                CSR_AFU_DSM_BASEH       Upper 32-bits of AFU DSM base address.
//     'h1A20:         WO                CSR_SRC_ADDR            Start physical address for source buffer. All read requests are targetted to this region.
//     'h1A24:         WO                CSR_DST_ADDR            Start physical address for destination buffer. All write requests are targetted to this region.
//     'h1A28:         WO                CSR_NUM_LINES           Number of cache lines
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
// CSR_NUM_LINES should be less than or equal to N.
//
// CSR_SRC_ADDR:
// [31:N]   WO   2^(N+6)MB aligned address points to the start of read buffer
// [N-1:0]  WO   'h0
//
// CSR_DST_ADDR:
// [31:N]   WO   2^(N+6)MB aligned address points to the start of write buffer
// [N-1:0]  WO   'h0
//
// CSR_NUM_LINES:
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
// CSR_NUM_LINES:
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
// [1]      WO    cr_cont      - 1- test rollsover to start address after it reaches the CSR_NUM_LINES count. Such a test terminates only on an error.
//                               0- test terminates, updated the status csr when CSR_NUM_LINES count is reached.
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

module bwa_mem_sw #(parameter TXHDR_WIDTH=61, RXHDR_WIDTH=18, DATA_WIDTH =512)
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
    rb2cf_C0RxUMsgValid,              //                     cci_intf:           Rx UMsg valid
    rb2cf_C0RxIntrValid,                //                     cci_intf:           Rx interrupt valid
    rb2cf_C1RxHdr,                    // [RXHDR_WIDTH-1:0]   cci_intf:           Rx header to SPL channel 1
    rb2cf_C1RxWrValid,                //                     cci_intf:           Rx write response valid
    rb2cf_C1RxIntrValid,                //                     cci_intf:           Rx interrupt valid

    cf2ci_C0TxHdr,                    // [TXHDR_WIDTH-1:0]   cci_intf:           Tx Header from SPL channel 0
    cf2ci_C0TxRdValid,                //                     cci_intf:           Tx read request enable
    cf2ci_C1TxHdr,                    //                     cci_intf:           Tx Header from SPL channel 1
    cf2ci_C1TxData,                   //                     cci_intf:           Tx data from SPL
    cf2ci_C1TxWrValid,                //                     cci_intf:           Tx write request enable
    cf2ci_C1TxIntrValid,              //                     cci_intf:           Tx interrupt valid
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
    input                        rb2cf_C0RxUMsgValid;  //                  cci_intf:           Rx UMsg valid
    input                        rb2cf_C0RxIntrValid;    //                  cci_intf:           interrupt response enable
    input [RXHDR_WIDTH-1:0]      rb2cf_C1RxHdr;        // [RXHDR_WIDTH-1:0]cci_intf:           Rx header to SPL channel 1
    input                        rb2cf_C1RxWrValid;    //                  cci_intf:           write response valid
    input                        rb2cf_C1RxIntrValid;    //                  cci_intf:           interrupt response valid

    output [TXHDR_WIDTH-1:0]     cf2ci_C0TxHdr;        // [TXHDR_WIDTH-1:0]cci_intf:           Tx Header from SPL channel 0
    output                       cf2ci_C0TxRdValid;    //                  cci_intf:           Tx read request enable
    output [TXHDR_WIDTH-1:0]     cf2ci_C1TxHdr;        //                  cci_intf:           Tx Header from SPL channel 1
    output [DATA_WIDTH -1:0]     cf2ci_C1TxData;       //                  cci_intf:           Tx data from SPL
    output                       cf2ci_C1TxWrValid;    //                  cci_intf:           Tx write request enable
    output                       cf2ci_C1TxIntrValid;  //                  cci_intf:           Tx interrupt valid
    input                        ci2cf_C0TxAlmFull;    //                  cci_intf:           Tx memory channel 0 almost full
    input                        ci2cf_C1TxAlmFull;    //                  cci_intf:           TX memory channel 1 almost full

    input                        ci2cf_InitDn;         //                  cci_intf:           Link initialization is complete

    localparam  NUM_PEA             = 4;
    localparam  TBB_WR_ADDR_WIDTH   = 12;
    localparam  TBB_WR_DATA_WIDTH   = 512;
    localparam  TBB_RD_ADDR_WIDTH   = 16;
    localparam  TBB_RD_DATA_WIDTH   = 32;
    localparam  RBB_WR_ADDR_WIDTH   = 12;
    localparam  RBB_WR_DATA_WIDTH   = 32;
    localparam  RBB_RD_ADDR_WIDTH   = 8;
    localparam  RBB_RD_DATA_WIDTH   = 512;

//   localparam      PEND_THRESH = 7;
//   localparam      ADDR_LMT    = 20;
//   localparam      MDATA       = 'd11;
   
//   wire                         re2xy_go;
//   wire [31:0]                  re2xy_src_addr;
//   wire [31:0]                  re2xy_dst_addr;
//   wire [31:0]                  re2xy_NumLines;
//   wire                         re2xy_Cont;
//   wire [7:0]                   re2xy_test_cfg;
//   wire [2:0]                   re2ab_Mode;
//   wire                         ab2re_TestCmp;
//   wire [255:0]                 ab2re_ErrorInfo;
//   wire                         ab2re_ErrorValid;
//   
//   wire                         test_Resetb;

    assign cf2ci_C1TxIntrValid = 'b0;

    batch_manager #(.TBB_WR_ADDR_WIDTH(TBB_WR_ADDR_WIDTH),
                    .TBB_WR_DATA_WIDTH(TBB_WR_DATA_WIDTH),
                    .TBB_RD_ADDR_WIDTH(TBB_RD_ADDR_WIDTH),
                    .TBB_RD_DATA_WIDTH(TBB_RD_DATA_WIDTH),
                    .RBB_WR_ADDR_WIDTH(RBB_WR_ADDR_WIDTH),
                    .RBB_WR_DATA_WIDTH(RBB_WR_DATA_WIDTH),
                    .RBB_RD_ADDR_WIDTH(RBB_RD_ADDR_WIDTH),
                    .RBB_RD_DATA_WIDTH(RBB_RD_DATA_WIDTH),
                    .NUM_PEA(NUM_PEA),
                    .TXHDR_WIDTH(TXHDR_WIDTH),
                    .RXHDR_WIDTH(RXHDR_WIDTH),
                    .DATA_WIDTH (DATA_WIDTH )
                    )
    batch_manager(
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

        ci2cf_InitDn                      // Link initialization is complete
    );

endmodule
