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

    //----------------------------------------------------------------------------------------------------------------------
    // NLB v1.1 AFU ID
    localparam       BWA_MEM_SW          = 128'h2015_0212_900d_beef_0000_0000_0000_0000;
    localparam       VERSION             = 16'h0001;
    
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

    //Question: Why not aligned?
    localparam      DEF_DSM_BASE         = 32'h04ff_ffff;           // default status address
    
    //---------------------------------------------------------
    // CSR Address Map ***** DO NOT MODIFY *****
    //---------------------------------------------------------
    localparam      CSR_AFU_DSM_BASEL    = 16'h1a00;                 // WO - Lower 32-bits of AFU DSM base address. The lower 6-bbits are 4x00 since the address is cache aligned.
    localparam      CSR_AFU_DSM_BASEH    = 16'h1a04;                 // WO - Upper 32-bits of AFU DSM base address.
    localparam      CSR_SRC_ADDR         = 16'h1a20;                 // WO   Reads are targetted to this region 
    localparam      CSR_DST_ADDR         = 16'h1a24;                 // WO   Writes are targetted to this region
    localparam      CSR_NUM_BATCHES        = 16'h1a28;                 // WO   Numbers of task batches to be read/write
    localparam      CSR_CTL              = 16'h1a2c;                 // WO   Control CSR to start n stop the test
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
  
    wire [NUM_PEA-1:0]			tbbWrEn_b;
    wire [TBB_ADDR_WIDTH*NUM_PEA-1:0]	tbbWrAddr_b;
    wire [TBB_DATA_WIDTH*NUM_PEA-1:0]	tbbWrDin_b;
    wire [NUM_PEA-1:0]			tbbFull_b;
 
    wire [NUM_PEA-1:0]			tbbRdEn_b;
    wire [TBB_ADDR_WIDTH*NUM_PEA-1:0]	tbbRdAddr_b;
    wire [TBB_DATA_WIDTH*NUM_PEA-1:0]	tbbRdDout_b;
    wire [NUM_PEA-1:0]			tbbEmpty_b;

    wire [NUM_PEA-1:0]			rbbWrEn_b;
    wire [RBB_ADDR_WIDTH*NUM_PEA-1:0]	rbbWrAddr_b;
    wire [RBB_DATA_WIDTH*NUM_PEA-1:0]	rbbWrDin_b;
    wire [NUM_PEA-1:0]			rbbFull_b;
 
    wire [NUM_PEA-1:0]			rbbRdEn_b;
    wire [RBB_ADDR_WIDTH*NUM_PEA-1:0]	rbbRdAddr_b;
    wire [RBB_DATA_WIDTH*NUM_PEA-1:0]	rbbRdDout_b;
    wire [NUM_PEA-1:0]			rbbEmpty_b;
    
    reg  [DATA_WIDTH-1:0]   cf2ci_C1TxData;
    reg  [TXHDR_WIDTH-1:0]  cf2ci_C1TxHdr;
    reg                     cf2ci_C1TxWrValid;
    reg  [TXHDR_WIDTH-1:0]  cf2ci_C0TxHdr;
    reg                     cf2ci_C0TxRdValid;
    //reg                     cf2ci_C1TxIntrValid;
    
    reg                     dsm_base_valid;
    reg                     afuid_updtd;
    
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
    //wire                    test_reset_n     = cr_ctl[0];                // Clears all the states. Either is one then test is out of Reset.
    wire                    test_go         = cr_ctl[1];                // When 0, it allows reconfiguration of test parameters.

    //register for storing number of task batches that get received
    reg [31:0]			NumBatchesRecv;
    //pointer to TBB
    reg [NUM_PEA-1:0]		tbb_pointer;
    //pointer to RBB
    reg [NUM_PEA-1:0]		rbb_pointer;

    //CCI Read Address Offset
    reg [TBB_ADDR_WIDTH-1:0]	RdAddrOffset;
    //CCI Read ID
    reg [13:0]			RdReqId;
    //CCI Read Type
    wire [3:0]			rdreq_type = RdLine;

    //CCI Write Address Offset
    reg [RBB_ADDR_WIDTH-1:0]	WrAddrOffset;
    //CCI Write ID
    reg [13:0]			WrReqId;
    //CCI Write Type
    wire [3:0]			wrreq_type = WrLine;

    wire   [31:0]            ds_afuid_address = dsm_offset2addr(DSM_AFU_ID,cr_dsm_base);                        // 0h - afu id is written to this address
    wire   [31:0]            ds_stat_address = dsm_offset2addr(DSM_STATUS,cr_dsm_base);                        // 40h - test status is written to this address
    wire                     re2xy_go = test_go & afuid_updtd & ci2cf_InitDn;					// After initializing DSM, we can do actual tasks on AFU
    reg			     WrHdr_valid;									// 1: Valid Write Request
    reg			     RdHdr_valid;									// 1: Valid Read Request

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
	    cr_num_batches  <= 0;
            cr_ctl          <= 0;
            dsm_base_valid  <= 0;
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
                                         dsm_base_valid     <= 1;
                                      end
                endcase
            end
            if(re2xy_go) // Execution Mode, following CSRs can only be updated in this mode
            begin
                if(rb2cf_C0RxCfgValid)
                case({rb2cf_C0RxHdr[13:0],2'b00})         /* synthesis parallel_case */
		    //cr_num_batches corresponds to the number of task batches available for processing
                    CSR_NUM_BATCHES:     cr_num_batches     <= cr_num_batches + 1;
                endcase
            end
        end
    end

    //-------------------------
    //Round-Robin Load/Store
    //-------------------------

    //Sequential Logic
    always @ (posedge clk) 
    begin
	    if (!reset_n)
	    begin
		    NumBatchesRecv <= 0;
		    tbb_pointer <= 0;
		    rbb_pointer <= 0;
		    RdAddrOffset <= 0;
		    WrAddrOffset <= 0;
	    end
	    else
	    begin
		    NumBatchesRecv <= NumBatchesRecv_d;
		    tbb_pointer <= tbb_pointer_d;
		    rbb_pointer <= rbb_pointer_d;
		    RdAddrOffset <= RdAddrOffset_d;
		    WrAddrOffset <= WrAddrOffset_d;
	    end
    end

    //Combinatorial Logic
    always @ (*) 
    begin
	    tbb_pointer_d = tbb_pointer;
	    rbb_pointer_d = rbb_pointer;
	    RdAddrOffset_d = RdAddrOffset;
	    WrAddrOffset_d = WrAddrOffset;
	    NumBatchesRecv_d = NumBatchesRecv;
	    RdHdr_valid = 0;
	    WrHdr_valid = 0;
	    tbbReqAck_b = 0;
	    rbbReqAck_b = 0;
	    RdReqId = 0;
	    WrReqId = 0;
	    if (re2xy_go) //During the real execution state, do the modification on these registers
	    begin
		    //tbb handler
		    //If current tbb is full, jump it without doing anything
		    if (tbbFull_b[tbb_pointer])
		    begin
			    if (tbb_pointer == (NUM_PEA-1))
				    tbb_pointer_d = 0;
			    else
				    tbb_pointer_d = tbb_pointer + 1;
		    end
		    //If tbb is not full, it can store a valid task
		    //Then, check if there is a valid task to process
		    else if (NumBatchesRecv != cr_num_batches)
		    begin
			    //We have at least one available batch to process
			    //Then, check if the TBB is willing to read data
			    if (tbbReqValid_b[tbb_pointer] && !ci2cf_C0TxAlmFull) //Not full, Available Batch, Request Valid and CCI not stalled, then just fetch data
			    begin
				    RdHdr_valid = 1; // a valid read request, 100% sent
				    RdAddrOffset_d = RdAddrOffset + 64; // update address since the current one has been sent
				    RdReqId = {tbb_pointer[13-TBB_LINE_IDX_WIDTH:0], tbbReqLineIdx_b[tbb_pointer*TBB_LINE_IDX_WIDTH+TBB_LINE_IDX_WIDTH-1:tbb_pointer*TBB_LINE_IDX_WIDTH]}; //Request ID is generated by combining the pointer and the line index
				    tbbReqAck_b[tbb_pointer] = 1; // tell the TBB that the request has been sent to CCI
			    end
			    //if TBB is willing to read data but CCI is stalled, then do nothing
			    else if (tbbReqValid_b[tbb_pointer])
			    begin
				    //do nothing
			    end
			    //if TBB is not willing to read data, then we can infer that the TBB has sent all the read requests
			    //We can go to next TBB, and meanwhile claim that a batch has been received (since all the requests of a batch have been sent to CCI)
			    else
			    begin
				    if (tbb_pointer == (NUM_PEA-1))
					    tbb_pointer_d = 0;
				    else
					    tbb_pointer_d = tbb_pointer + 1;
				    NumBatchesRecv_d = NumBatchesRecv + 1;
			    end
		    end

		    //rbb handler
		    //If current rbb is empty, jump it without doing anything
		    if (rbbEmpty_b[rbb_pointer])
		    begin
			    if (rbb_pointer == (NUM_PEA-1))
				    rbb_pointer_d = 0;
			    else
				    rbb_pointer_d = rbb_pointer + 1;
		    end
		    //If current rbb is not empty, we are able to send data back to CCI
		    else
		    begin
			    //check if the RBB is willing to write data
			    if (rbbReqValid_b[rbb_pointer] && !ci2cf_C1TxAlmFull) //Not empty, Request Valid and CCI not stalled, then just write back data
			    begin
				    WrHdr_valid = 1; // a valid write request, 100% sent
				    WrAddrOffset_d = WrAddrOffset + 64; // update address since the current one has been sent
				    WrReqId = {rbb_pointer[13-RBB_LINE_IDX_WIDTH:0], rbbReqLineIdx_b[rbb_pointer*RBB_LINE_IDX_WIDTH+RBB_LINE_IDX_WIDTH-1:rbb_pointer*RBB_LINE_IDX_WIDTH]}; //Request ID is generated by combining the pointer and the line index
				    rbbReqAck_b[rbb_pointer] = 1; // tell the RBB that the request has been sent to CCI
			    end
			    //if RBB is willing to write data but CCI is stalled, then do nothing
			    else if (rbbReqValid_b[rbb_pointer])
			    begin
				    //do nothing
			    end
			    //if RBB is not willing to write data, then we can infer that the RBB has sent all the write requests
			    //We can go to next RBB, and meanwhile claim that a result batch has been sent back
			    //Question: How to let CPU know it?
			    else
			    begin
				    if (rbb_pointer == (NUM_PEA-1))
					    rbb_pointer_d = 0;
				    else
					    rbb_pointer_d = rbb_pointer + 1;
			    end
		    end
	    end
    end

    //-------------------------
    //Handle CCI Tx Channels
    //-------------------------
    
        // Format Read Header
    wire [31:0]             RdAddr  = cr_src_address ^ RdAddrOffset;
    wire [TXHDR_WIDTH-1:0]  RdHdr   = {
                                        5'h00,                          // [60:56]      Byte Enable
                                        rdreq_type,                     // [55:52]      Request Type
                                        6'h00,                          // [51:46]      Rsvd
                                        RdAddr,                         // [45:14]      Address
                                        RdReqId                         // [13:0]       Meta data to track the SPL requests
                                      };
    
        // Format Write Header
    wire [31:0]             WrAddr      = cr_dst_address ^ WrAddrOffset;
    wire [DATA_WIDTH-1:0]   WrData      = rbbRdDout_b[DATA_WIDTH-1+DATA_WIDTH*rbb_pointer:DATA_WIDTH*rbb_pointer];
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
        if(!reset_n)
        begin
            afuid_updtd             <= 0;
            cf2ci_C1TxHdr           <= 0;
            cf2ci_C1TxWrValid       <= 0;
	    cf2ci_C1TxData	    <= 0;
            //cf2ci_C1TxIntrValid     <= 0;
            cf2ci_C0TxHdr           <= 0;
            cf2ci_C0TxRdValid       <= 0;
        end
	else
	begin	
            //Tx Path
            //--------------------------------------------------------------------------

            // Channel 1
            if(ci2cf_C1TxAlmFull==0)
            begin
		//The first write request should be DSM initialization
                if( ci2cf_InitDn && dsm_base_valid && !afuid_updtd )
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
                else if (re2xy_go)	//Executing real tasks
                begin
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
            if(  re2xy_go 
              && RdHdr_valid && !ci2cf_C0TxAlmFull )                                // Read from Source Workspace
            begin                                                                   //----------------------------------
                cf2ci_C0TxHdr      <= RdHdr;
                cf2ci_C0TxRdValid  <= 1;
                //Num_Reads          <= Num_Reads + 1'b1;
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
    assign tbbWrEn_b[rb2cf_C0RxHdr[13:TBB_LINE_IDX_WIDTH]] = re2xy_go && rb2cf_C0RxRdValid;
    assign tbbWrDin_b[rb2cf_C0RxHdr[13:TBB_LINE_IDX_WIDTH]*DATA_WIDTH+DATA_WIDTH-1:rb2cf_C0RxHdr[13:TBB_LINE_IDX_WIDTH]*DATA_WIDTH] = rb2cf_C0RxData;
    assign tbbWrAddr_b[TBB_ADDR_WIDTH*rb2cf_C0RxHdr[13:TBB_LINE_IDX_WIDTH]+TBB_ADDR_WIDTH-1:TBB_ADDR_WIDTH*rb2cf_C0RxHdr[13:TBB_LINE_IDX_WIDTH]] = rb2cf_C0RxHdr[TBB_LINE_IDX_WIDTH-1:0];

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

    //-------------------------
    //TBB Generation
    //-------------------------

generate
  genvar i;
  for (i=0; i<NUM_PEA; i=i+1) begin
	  tbb tbb(
	     .clk					(clk),
	     .reset_n					(reset_n),
	     .WrEn 	         			(tbbWrEn_b[i]),
    	     .WrAddr					(tbbWrAddr_b[i*TBB_ADDR_WIDTH+TBB_ADDR_WIDTH-1:i*TBB_ADDR_WIDTH]),
    	     .WrDin					(tbbWrDin_b[i*TBB_DATA_WIDTH+TBB_DATA_WIDTH-1:i*TBB_DATA_WIDTH]),
    	     .Full 					(tbbFull_b[i]),
    	     .RdEn 					(tbbRdEn_b[i]),
    	     .RdAddr					(tbbRdAddr_b[i*TBB_ADDR_WIDTH+TBB_ADDR_WIDTH-1:i*TBB_ADDR_WIDTH]),
    	     .RdDout					(tbbRdDout_b[i*TBB_DATA_WIDTH+TBB_DATA_WIDTH-1:i*TBB_DATA_WIDTH]),
    	     .Empty 					(tbbEmpty_b[i])
	  ); 
  end
endgenerate

    //-------------------------
    //RBB Generation
    //-------------------------

generate
  genvar i;
  for (i=0; i<NUM_PEA; i=i+1) begin
	  rbb rbb(
	     .clk					(clk),
	     .reset_n					(reset_n),
	     .WrEn 	         			(rbbWrEn_b[i]),
    	     .WrAddr					(rbbWrAddr_b[i*RBB_ADDR_WIDTH+RBB_ADDR_WIDTH-1:i*RBB_ADDR_WIDTH]),
    	     .WrDin					(rbbWrDin_b[i*RBB_DATA_WIDTH+RBB_DATA_WIDTH-1:i*RBB_DATA_WIDTH]),
    	     .Full 					(rbbFull_b[i]),
    	     .RdEn 					(rbbRdEn_b[i]),
    	     .RdAddr					(rbbRdAddr_b[i*RBB_ADDR_WIDTH+RBB_ADDR_WIDTH-1:i*RBB_ADDR_WIDTH]),
    	     .RdDout					(rbbRdDout_b[i*RBB_DATA_WIDTH+RBB_DATA_WIDTH-1:i*RBB_DATA_WIDTH]),
    	     .Empty 					(rbbEmpty_b[i])
	  ); 
  end
endgenerate

endmodule

