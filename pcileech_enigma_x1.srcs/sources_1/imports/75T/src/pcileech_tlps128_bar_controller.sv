
`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register_0,
    output wire             int_enable,  
    output wire [63:0]      msi_address,
    output wire [31:0]      msi_vector,
    output wire             o_msix_req,
    input wire  [31:0]      debug_status,
    IfPCIeSignals.mpm       ctx,   
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out,
    output wire [7:0]       bar0_debug_out // <-- AÑADE ESTA LÍNEA

);
   

    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
    
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx_0;
    wire [31:0] bar_rsp_data_0;
    wire        bar_rsp_valid_0;
    wire        is_debug_read = rd_req_valid && rd_req_bar[0] && ((rd_req_addr & 32'hFFF) == 32'hFFC);

    
    assign rd_rsp_ctx = is_debug_read ? rd_req_ctx :
                         bar_rsp_valid_0 ? bar_rsp_ctx_0 : 0;

    // Simplificación para rd_rsp_data
    assign rd_rsp_data = is_debug_read ? {24'h0, bar0_debug_out} :
                         bar_rsp_valid_0 ? bar_rsp_data_0 : 0;
    
    // Simplificación para rd_rsp_valid
    assign rd_rsp_valid = is_debug_read || bar_rsp_valid_0;

    wire w_msix_req;                      // <-- 1. DECLARA EL WIRE
    assign o_msix_req = w_msix_req;       // <-- 2. CONÉCTALO A LA NUEVA SALIDA
    
    pcileech_bar_impl_bar0 i_bar0(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register( base_address_register_0 ),
        .int_enable     ( int_enable                    ),
        .rd_rsp_ctx     ( bar_rsp_ctx_0                 ),
        .rd_rsp_data    ( bar_rsp_data_0                ),
        .rd_rsp_valid   ( bar_rsp_valid_0               ),
        .msi_address    ( msi_address                   ),
        .msi_vector     ( msi_vector                    ),
        .msix_req       ( w_msix_req                    ),
        .bar0_debug_out ( bar0_debug_out                ) // <-- AÑADE ESTA LÍNEA


);

endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);

    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;
    
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule


module pcileech_bar_impl_bar0 (
    input               rst,
    input               clk,
    input      [31:0]   wr_addr,
    input      [3:0]    wr_be,
    input      [31:0]   wr_data,
    input               wr_valid,
    input      [87:0]   rd_req_ctx,
    input      [31:0]   rd_req_addr,
    input               rd_req_valid,
    output wire [7:0]   bar0_debug_out,
    input      [31:0]   base_address_register,
    output wire         int_enable,
    output reg [63:0]   msi_address,
    output reg [31:0]   msi_vector,
    output reg          msix_req,
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);

    //======================================================================
    // 1. REGISTROS DE PIPELINE Y DE ESTADO
    //======================================================================

    reg [87:0]  drd_req_ctx;
    reg [31:0]  drd_req_addr;
    reg         drd_req_valid;
    reg [31:0]  dwr_addr;
    reg [31:0]  dwr_data;
    reg         dwr_valid;

    // --- Registros de Estado xHCI ---
    reg [31:0]  usbcmd_reg;
    reg [31:0]  usbsts_reg;
    reg [63:0]  crcr_reg;
    reg [63:0]  dcbaap_reg;
    reg [31:0]  config_reg;
    
    // Registros del Interrupter 0
    reg [31:0]  iman0_reg;
    reg [31:0]  imod0_reg;
    reg [31:0]  erstsz0_reg;
    reg [63:0]  erstba0_reg;
    reg [63:0]  erdp0_reg;

    // --- Lógica de Handshake para el Reset ---
    reg         is_in_hcrst;
    reg [7:0]   hcrst_counter;
    
    // Puertos 1-8 (USB 3.0)
    reg [31:0]  portsc1_reg, portsc2_reg, portsc3_reg, portsc4_reg, portsc5_reg, portsc6_reg, portsc7_reg, portsc8_reg;
    reg [31:0]  portpmsc1_reg, portpmsc2_reg, portpmsc3_reg, portpmsc4_reg, portpmsc5_reg, portpmsc6_reg, portpmsc7_reg, portpmsc8_reg;
    reg [31:0]  portli1_reg, portli2_reg, portli3_reg, portli4_reg, portli5_reg, portli6_reg, portli7_reg, portli8_reg;
    reg [31:0]  reg_0x418;
    reg [31:0]  dummy_doorbell_reg;
    
    reg [63:0] msix_addr_table [0:7];
    reg [31:0] msix_data_table [0:7];
    reg [31:0] msix_ctrl_table [0:7];
    reg trigger_port1_change_event;
    wire hcrst_write_req = dwr_valid && (((dwr_addr - (base_address_register & 32'hFFFFE000)) & 32'h1FFF) == 16'h0020) && dwr_data[1];

    assign int_enable = (~msix_ctrl_table[0][0]) | (~msix_ctrl_table[1][0]) | (~msix_ctrl_table[2][0]) | (~msix_ctrl_table[3][0]) | (~msix_ctrl_table[4][0]) | (~msix_ctrl_table[5][0]) | (~msix_ctrl_table[6][0]) | (~msix_ctrl_table[7][0]);
    integer i;
    
always @(posedge clk) begin
    if (rst) begin
        // --- Reset de la lógica de interrupción ---
        msix_req <= 1'b0;
        msi_address <= 64'h0;
        msi_vector <= 32'h0;
        trigger_port1_change_event <= 1'b0;

        // --- Reset de la lógica de registros xHCI ---
        usbcmd_reg  <= 32'h0;
        usbsts_reg <= 32'h00001801; // CNR (bit 11) = 1, HCHalted (bit 0) = 1
        is_in_hcrst <= 1'b0;
        hcrst_counter<= 8'h0;
        crcr_reg    <= 64'h0;
        dcbaap_reg  <= 64'h0;
        config_reg  <= 32'h0;
        iman0_reg   <= 32'h0;
        imod0_reg   <= 32'h00000FA0;
        erstsz0_reg <= 32'h0;
        erstba0_reg <= 64'h0;
        erdp0_reg   <= 64'h0;
        portsc1_reg <= 32'h000002A0;
        portsc2_reg <= 32'h000002A0;
        portsc3_reg <= 32'h000002A0;
        portsc4_reg <= 32'h000002A0;
        portsc5_reg <= 32'h000002A0;
        portsc6_reg <= 32'h000002A0;
        portsc7_reg <= 32'h000002A0;
        portsc8_reg <= 32'h000002A0; 
        reg_0x418   <= 32'h00000001;
        for (i = 0; i < 8; i = i + 1) begin
            msix_addr_table[i] <= 64'h0;
            msix_data_table[i] <= 32'h0;
            msix_ctrl_table[i] <= 32'h00000001;
        end

    end else begin
        
        // --- 1. Lógica de Pipeline (siempre se ejecuta) ---
        drd_req_ctx   <= rd_req_ctx;
        drd_req_addr  <= rd_req_addr;
        drd_req_valid <= rd_req_valid;
        dwr_addr      <= wr_addr;
        dwr_data      <= wr_data;
        dwr_valid     <= wr_valid;
        
        // --- 2. Lógica de Generación de Interrupciones (con prioridad) ---
        msix_req <= 1'b0; // Valor por defecto

        // Prioridad 1: Evento de hardware (automático, vector 0)
        if (trigger_port1_change_event) begin
            msix_req    <= 1'b1;
            msi_address <= msix_addr_table[0];
            msi_vector  <= msix_data_table[0];
            portsc1_reg[17] <= 1'b1;
            trigger_port1_change_event <= 1'b0;
        // Prioridad 2: Escritura en el timbre (respuesta al driver, vector 1)
        end else if (dwr_valid && (((dwr_addr - (base_address_register & 32'hFFFFE000)) & 32'h1FFF) == 16'h0600)) begin
            msix_req    <= 1'b1;
            msi_address <= msix_addr_table[1];
            msi_vector  <= msix_data_table[1];
        end 

        // --- 3. Máquina de Estados Principal (Reset y Escritura de Registros) ---
        if (hcrst_write_req && !is_in_hcrst) begin
            is_in_hcrst   <= 1'b1;
            hcrst_counter <= 8'd20;
            usbcmd_reg    <= 32'h00000002;
            usbsts_reg    <= 32'h00001800;
            crcr_reg      <= 64'h0;
            dcbaap_reg    <= 64'h0;
            config_reg    <= 32'h0;
            iman0_reg     <= 32'h0;
            imod0_reg     <= 32'h00000FA0;
            erstsz0_reg   <= 32'h0;
            erstba0_reg   <= 64'h0;
            erdp0_reg     <= 64'h0;
        end else if (is_in_hcrst) begin
            if (hcrst_counter > 0) begin
                hcrst_counter <= hcrst_counter - 1;
                if (hcrst_counter == 1) begin
                    trigger_port1_change_event <= 1'b1;
                end
            end else begin
                is_in_hcrst   <= 1'b0;
                usbcmd_reg[1] <= 1'b0;
                usbsts_reg[11]<= 1'b0; // **NUEVA LÍNEA: Pone CNR a 0**
            end
        end else if (dwr_valid) begin
            // --- Lógica de Escritura Normal (cuando no hay reset) ---
            case (((dwr_addr - (base_address_register & 32'hFFFFE000)) & 32'h1FFF))
                16'h0020: usbcmd_reg <= dwr_data;
                16'h0024: usbsts_reg <= usbsts_reg & ~dwr_data;
                16'h0038: crcr_reg[31:0]   <= dwr_data;
                16'h003C: crcr_reg[63:32]  <= dwr_data;
                16'h0050: dcbaap_reg[31:0] <= dwr_data;
                16'h0054: dcbaap_reg[63:32]<= dwr_data;
                16'h0058: config_reg       <= dwr_data;
                16'h0418: reg_0x418        <= dwr_data;
                16'h0420: portsc1_reg <= (portsc1_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0424: portpmsc1_reg <= dwr_data;
                16'h0428: portli1_reg <= dwr_data;
                16'h0430: portsc2_reg <= (portsc2_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0434: portpmsc2_reg <= dwr_data;
                16'h0438: portli2_reg <= dwr_data;
                16'h0440: portsc3_reg <= (portsc3_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0444: portpmsc3_reg <= dwr_data;
                16'h0448: portli3_reg <= dwr_data;
                16'h0450: portsc4_reg <= (portsc4_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0454: portpmsc4_reg <= dwr_data;
                16'h0458: portli4_reg <= dwr_data;
                16'h0460: portsc5_reg <= (portsc5_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0464: portpmsc5_reg <= dwr_data;
                16'h0468: portli5_reg <= dwr_data;
                16'h0470: portsc6_reg <= (portsc6_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0474: portpmsc6_reg <= dwr_data;
                16'h0478: portli6_reg <= dwr_data;
                16'h0480: portsc7_reg <= (portsc7_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0484: portpmsc7_reg <= dwr_data;
                16'h0488: portli7_reg <= dwr_data;
                16'h0490: portsc8_reg <= (portsc8_reg & ~32'h00FE0002) | (dwr_data & 32'h00FE0002) & ~(dwr_data & 32'h00FE0000);
                16'h0494: portpmsc8_reg <= dwr_data;
                16'h0498: portli8_reg <= dwr_data;
                16'h0620: iman0_reg        <= dwr_data;
                16'h0624: imod0_reg        <= dwr_data;
                16'h0628: erstsz0_reg      <= dwr_data;
                16'h0630: erstba0_reg[31:0]<= dwr_data;
                16'h0634: erstba0_reg[63:32]<= dwr_data;
                16'h0638: erdp0_reg[31:0]  <= dwr_data;
                16'h063C: erdp0_reg[63:32] <= dwr_data;
                16'h1000: msix_addr_table[0][31:0] <= dwr_data;
                16'h1004: msix_addr_table[0][63:32] <= dwr_data;
                16'h1008: msix_data_table[0] <= dwr_data;
                16'h100C: msix_ctrl_table[0] <= dwr_data;
                16'h1010: msix_addr_table[1][31:0] <= dwr_data;
                16'h1014: msix_addr_table[1][63:32] <= dwr_data;
                16'h1018: msix_data_table[1] <= dwr_data;
                16'h101C: msix_ctrl_table[1] <= dwr_data;
                16'h1020: msix_addr_table[2][31:0] <= dwr_data;
                16'h1024: msix_addr_table[2][63:32] <= dwr_data;
                16'h1028: msix_data_table[2] <= dwr_data;
                16'h102C: msix_ctrl_table[2] <= dwr_data;
                16'h1030: msix_addr_table[3][31:0] <= dwr_data;
                16'h1034: msix_addr_table[3][63:32] <= dwr_data;
                16'h1038: msix_data_table[3] <= dwr_data;
                16'h103C: msix_ctrl_table[3] <= dwr_data;
                16'h1040: msix_addr_table[4][31:0] <= dwr_data;
                16'h1044: msix_addr_table[4][63:32] <= dwr_data;
                16'h1048: msix_data_table[4] <= dwr_data;
                16'h104C: msix_ctrl_table[4] <= dwr_data;
                16'h1050: msix_addr_table[5][31:0] <= dwr_data;
                16'h1054: msix_addr_table[5][63:32] <= dwr_data;
                16'h1058: msix_data_table[5] <= dwr_data;
                16'h105C: msix_ctrl_table[5] <= dwr_data;
                16'h1060: msix_addr_table[6][31:0] <= dwr_data;
                16'h1064: msix_addr_table[6][63:32] <= dwr_data;
                16'h1068: msix_data_table[6] <= dwr_data;
                16'h106C: msix_ctrl_table[6] <= dwr_data;
                16'h1070: msix_addr_table[7][31:0] <= dwr_data;
                16'h1074: msix_addr_table[7][63:32] <= dwr_data;
                16'h1078: msix_data_table[7] <= dwr_data;
                16'h107C: msix_ctrl_table[7] <= dwr_data;
                default: ;
            endcase
        end
        
        // --- 4. Lógica de Estado Continua y Lectura ---
        if (!is_in_hcrst) begin
            usbsts_reg[0] <= ~usbcmd_reg[0];
        end
        
        // --- Lógica de Lectura y Respuesta ---
        rd_rsp_valid <= drd_req_valid;
        rd_rsp_ctx   <= drd_req_ctx;

        if (drd_req_valid) begin
            case (((drd_req_addr - (base_address_register & 32'hFFFFE000)) & 32'h1FFF))
                16'h0000: rd_rsp_data <= 32'h01000020;
                16'h0004: rd_rsp_data <= 32'h08000822;
                16'h0008: rd_rsp_data <= 32'h24000011;
                16'h000C: rd_rsp_data <= 32'h00000000;
                16'h0010: rd_rsp_data <= 32'h014051CF;
                16'h0014: rd_rsp_data <= 32'h00000800;
                16'h0018: rd_rsp_data <= 32'h00000600;
                16'h0418: rd_rsp_data <= (dwr_valid && (((dwr_addr - (base_address_register & 32'hFFFFE000)) & 32'h1FFF) == 16'h0418)) ? dwr_data : 32'h00000001;
                16'h0020: rd_rsp_data <= usbcmd_reg;
                16'h0024: rd_rsp_data <= usbsts_reg;
                16'h0028: rd_rsp_data <= 32'h00000001;
                16'h0038: rd_rsp_data <= crcr_reg[31:0];
                16'h003C: rd_rsp_data <= crcr_reg[63:32];
                16'h0050: rd_rsp_data <= dcbaap_reg[31:0];
                16'h0054: rd_rsp_data <= dcbaap_reg[63:32];
                16'h0058: rd_rsp_data <= config_reg;
                16'h0420: rd_rsp_data <= portsc1_reg;
                16'h0424: rd_rsp_data <= portpmsc1_reg;
                16'h0428: rd_rsp_data <= portli1_reg;
                16'h042C: rd_rsp_data <= 32'h0;
                16'h0430: rd_rsp_data <= portsc2_reg;
                16'h0434: rd_rsp_data <= portpmsc2_reg;
                16'h0438: rd_rsp_data <= portli2_reg;
                16'h043C: rd_rsp_data <= 32'h0;
                16'h0440: rd_rsp_data <= portsc3_reg;
                16'h0444: rd_rsp_data <= portpmsc3_reg;
                16'h0448: rd_rsp_data <= portli3_reg;
                16'h044C: rd_rsp_data <= 32'h0;
                16'h0450: rd_rsp_data <= portsc4_reg;
                16'h0454: rd_rsp_data <= portpmsc4_reg;
                16'h0458: rd_rsp_data <= portli4_reg;
                16'h045C: rd_rsp_data <= 32'h0;
                16'h0460: rd_rsp_data <= portsc5_reg;
                16'h0464: rd_rsp_data <= portpmsc5_reg;
                16'h0468: rd_rsp_data <= portli5_reg;
                16'h046C: rd_rsp_data <= 32'h0;
                16'h0470: rd_rsp_data <= portsc6_reg;
                16'h0474: rd_rsp_data <= portpmsc6_reg;
                16'h0478: rd_rsp_data <= portli6_reg;
                16'h047C: rd_rsp_data <= 32'h0;
                16'h0480: rd_rsp_data <= portsc7_reg;
                16'h0484: rd_rsp_data <= portpmsc7_reg;
                16'h0488: rd_rsp_data <= portli7_reg;
                16'h048C: rd_rsp_data <= 32'h0;
                16'h0490: rd_rsp_data <= portsc8_reg;
                16'h0494: rd_rsp_data <= portpmsc8_reg;
                16'h0498: rd_rsp_data <= portli8_reg;
                16'h049C: rd_rsp_data <= 32'h0;
                16'h0620: rd_rsp_data <= iman0_reg;
                16'h0624: rd_rsp_data <= imod0_reg;
                16'h0628: rd_rsp_data <= erstsz0_reg;
                16'h0630: rd_rsp_data <= erstba0_reg[31:0];
                16'h0634: rd_rsp_data <= erstba0_reg[63:32];
                16'h0638: rd_rsp_data <= erdp0_reg[31:0];
                16'h063C: rd_rsp_data <= erdp0_reg[63:32];
                16'h0500: rd_rsp_data <= 32'h01000401;
                16'h0510: rd_rsp_data <= 32'h03000502;
                16'h0514: rd_rsp_data <= 32'h20425355;
                16'h0518: rd_rsp_data <= 32'h00000401;
                16'h0524: rd_rsp_data <= 32'h02000702;
                16'h0528: rd_rsp_data <= 32'h20425355;
                16'h052C: rd_rsp_data <= 32'h00000405;
                16'h1000: rd_rsp_data <= msix_addr_table[0][31:0];
                16'h1004: rd_rsp_data <= msix_addr_table[0][63:32];
                16'h1008: rd_rsp_data <= msix_data_table[0];
                16'h100C: rd_rsp_data <= msix_ctrl_table[0];
                16'h1010: rd_rsp_data <= msix_addr_table[1][31:0];
                16'h1014: rd_rsp_data <= msix_addr_table[1][63:32];
                16'h1018: rd_rsp_data <= msix_data_table[1];
                16'h101C: rd_rsp_data <= msix_ctrl_table[1];
                16'h1020: rd_rsp_data <= msix_addr_table[2][31:0];
                16'h1024: rd_rsp_data <= msix_addr_table[2][63:32];
                16'h1028: rd_rsp_data <= msix_data_table[2];
                16'h102C: rd_rsp_data <= msix_ctrl_table[2];
                16'h1030: rd_rsp_data <= msix_addr_table[3][31:0];
                16'h1034: rd_rsp_data <= msix_addr_table[3][63:32];
                16'h1038: rd_rsp_data <= msix_data_table[3];
                16'h103C: rd_rsp_data <= msix_ctrl_table[3];
                16'h1040: rd_rsp_data <= msix_addr_table[4][31:0];
                16'h1044: rd_rsp_data <= msix_addr_table[4][63:32];
                16'h1048: rd_rsp_data <= msix_data_table[4];
                16'h104C: rd_rsp_data <= msix_ctrl_table[4];
                16'h1050: rd_rsp_data <= msix_addr_table[5][31:0];
                16'h1054: rd_rsp_data <= msix_addr_table[5][63:32];
                16'h1058: rd_rsp_data <= msix_data_table[5];
                16'h105C: rd_rsp_data <= msix_ctrl_table[5];
                16'h1060: rd_rsp_data <= msix_addr_table[6][31:0];
                16'h1064: rd_rsp_data <= msix_addr_table[6][63:32];
                16'h1068: rd_rsp_data <= msix_data_table[6];
                16'h106C: rd_rsp_data <= msix_ctrl_table[6];
                16'h1070: rd_rsp_data <= msix_addr_table[7][31:0];
                16'h1074: rd_rsp_data <= msix_addr_table[7][63:32];
                16'h1078: rd_rsp_data <= msix_data_table[7];
                16'h107C: rd_rsp_data <= msix_ctrl_table[7];
                default: rd_rsp_data <= 32'h0;
            endcase
        end else begin
            rd_rsp_data <= 32'h0;
        end
    end
end

endmodule

