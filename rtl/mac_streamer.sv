/*
 * mac_streamer.sv
 * Francesco Conti <fconti@iis.ee.ethz.ch>
 *
 * Copyright (C) 2018 ETH Zurich, University of Bologna
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the "License"); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

import mac_package::*;
import hwpe_stream_package::*;

module mac_streamer
#(
  parameter int unsigned MP = 4, // number of master ports
  parameter int unsigned FD = 2,  // FIFO depth
  parameter DATA_WIDTH = 32
)
(
  // global signals
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   test_mode_i,
  // local enable & clear
  input  logic                   enable_i,
  input  logic                   clear_i,

  // input a stream + handshake
  hwpe_stream_intf_stream.source a_o,
  // input b stream + handshake
  hwpe_stream_intf_stream.source b_o,
  // input c stream + handshake
  hwpe_stream_intf_stream.source c_o,
  // output d stream + handshake
  hwpe_stream_intf_stream.sink   d_i,

  // TCDM ports
  hwpe_stream_intf_tcdm.master tcdm [MP-1:0],

  // control channel
  input  ctrl_streamer_t  ctrl_i,
  output flags_streamer_t flags_o
);

  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( DATA_WIDTH )
  ) a_prefifo (
    .clk ( clk_i )
  );
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( DATA_WIDTH )
  ) b_prefifo (
    .clk ( clk_i )
  );
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( DATA_WIDTH )
  ) c_prefifo (
    .clk ( clk_i )
  );
  hwpe_stream_intf_stream #(
    .DATA_WIDTH ( DATA_WIDTH )
  ) d_postfifo (
    .clk ( clk_i )
  );

  // 1 mux input per stream + a dummy mux input for dbg mode 
  hwpe_stream_intf_tcdm tcdm_muxed [4:0] (
    .clk ( clk_i )
  );

  hwpe_stream_intf_tcdm tcdm_0 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_1 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_2 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_3 [0:0] (
    .clk ( clk_i )
  );

  // source and sink modules
  hwpe_stream_source #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .DECOUPLED  ( 1  )
  ) i_a_source (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .tcdm               ( tcdm_0                 ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( a_prefifo.source/*a_o*/       ),
    .ctrl_i             ( ctrl_i.fsm_ctrl.a_source_ctrl   ),
    .flags_o            ( flags_o.a_source_flags ),
    .tcdm_fifo_ready_o  (       )
  );

  hwpe_stream_source #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .DECOUPLED  ( 1  )
  ) i_b_source (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .tcdm               ( tcdm_1                 ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( b_prefifo.source/*b_o*/       ),
    .ctrl_i             ( ctrl_i.fsm_ctrl.b_source_ctrl   ),
    .flags_o            ( flags_o.b_source_flags ),
    .tcdm_fifo_ready_o  (       )
  );

  hwpe_stream_source #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .DECOUPLED  ( 1  )
  ) i_c_source (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .tcdm               ( tcdm_2                 ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( c_prefifo.source/*c_o*/       ),
    .ctrl_i             ( ctrl_i.fsm_ctrl.c_source_ctrl   ),
    .flags_o            ( flags_o.c_source_flags ),
    .tcdm_fifo_ready_o  (       )
  );

  hwpe_stream_sink #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .USE_TCDM_FIFOS ( 0 )
  ) i_d_sink (
    .clk_i       ( clk_i                ),
    .rst_ni      ( rst_ni               ),
    .test_mode_i ( test_mode_i          ),
    .clear_i     ( clear_i              ),
    .tcdm        ( tcdm_3               ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream      ( d_postfifo.sink/*d_i*/      ),
    .ctrl_i      ( ctrl_i.fsm_ctrl.d_sink_ctrl   ),
    .flags_o     ( flags_o.d_sink_flags )
  );


always_comb begin
  tcdm_muxed[0].req       = tcdm_0[0].req;
  tcdm_muxed[0].add       = tcdm_0[0].add;
  tcdm_muxed[0].wen       = tcdm_0[0].wen;
  tcdm_muxed[0].be        = tcdm_0[0].be;
  tcdm_muxed[0].data      = tcdm_0[0].data;
  tcdm_0[0].gnt           = tcdm_muxed[0].gnt;
  tcdm_0[0].r_data        = tcdm_muxed[0].r_data;
  tcdm_0[0].r_valid       = tcdm_muxed[0].r_valid;

  tcdm_muxed[1].req       = tcdm_1[0].req;
  tcdm_muxed[1].add       = tcdm_1[0].add;
  tcdm_muxed[1].wen       = tcdm_1[0].wen;
  tcdm_muxed[1].be        = tcdm_1[0].be;
  tcdm_muxed[1].data      = tcdm_1[0].data;
  tcdm_1[0].gnt           = tcdm_muxed[1].gnt;
  tcdm_1[0].r_data        = tcdm_muxed[1].r_data;
  tcdm_1[0].r_valid       = tcdm_muxed[1].r_valid;

  tcdm_muxed[2].req       = tcdm_2[0].req;
  tcdm_muxed[2].add       = tcdm_2[0].add;
  tcdm_muxed[2].wen       = tcdm_2[0].wen;
  tcdm_muxed[2].be        = tcdm_2[0].be;
  tcdm_muxed[2].data      = tcdm_2[0].data;
  tcdm_2[0].gnt           = tcdm_muxed[2].gnt;
  tcdm_2[0].r_data        = tcdm_muxed[2].r_data;
  tcdm_2[0].r_valid       = tcdm_muxed[2].r_valid;

  tcdm_muxed[3].req       = tcdm_3[0].req;
  tcdm_muxed[3].add       = tcdm_3[0].add;
  tcdm_muxed[3].wen       = tcdm_3[0].wen;
  tcdm_muxed[3].be        = tcdm_3[0].be;
  tcdm_muxed[3].data      = tcdm_3[0].data;
  tcdm_3[0].gnt           = tcdm_muxed[3].gnt;
  tcdm_3[0].r_data        = tcdm_muxed[3].r_data;
  tcdm_3[0].r_valid       = tcdm_muxed[3].r_valid;

  // dummy inputs to TCDM in dbg mode
  tcdm_muxed[4].req       = '0;
  tcdm_muxed[4].add       = '0;
  tcdm_muxed[4].wen       = '0;
  tcdm_muxed[4].be        = '0;
  tcdm_muxed[4].data      = '0;

end

  assign flags_o.a_addr = tcdm_muxed[0].add;
  assign flags_o.b_addr = tcdm_muxed[1].add;
  assign flags_o.c_addr = tcdm_muxed[2].add;
  assign flags_o.d_addr = tcdm_muxed[3].add;


  typedef enum {A=0, B=1, C=2, D=3, DBG=4} tcdm_id;
  tcdm_id req_sel_q, req_sel_n, resp_sel_q, req_sel_n_reg;   // mux select signals
  logic [$clog2(MAC_CNT_LEN):0] cnt, r_cnt;

  always_comb begin
    cnt = r_cnt + 1'b1;
    if ( r_cnt == ctrl_i.fsm_ctrl.len ) begin
      cnt = '0;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni)
  begin : main_fsm_seq
    if(~rst_ni) begin
      r_cnt <= '0;
      req_sel_q <= A;
      resp_sel_q <= A;
      req_sel_n_reg <= A;
    end
    else if(clear_i) begin
      r_cnt <= '0;
      req_sel_q <= A;
      resp_sel_q <= A;
      req_sel_n_reg <= A;
    end
    else begin
      // If not in dbg mode or if in dbg_mode and dbg_step says so, then everythig progresses 
      if (~ctrl_i.dbg_active | ctrl_i.dbg_step) begin
        if ( (req_sel_q == A && req_sel_n == B) | (r_cnt == ctrl_i.fsm_ctrl.len && req_sel_q == D) ) begin
          r_cnt <= cnt;
        end else begin
          r_cnt <= r_cnt;
        end
        req_sel_q <= req_sel_n;
        // Response select signal is simply the delayed request signal
        // since r_valid is guaranteed the cycle after the grant
        resp_sel_q <= req_sel_q;
      end else begin
      // Otherwise everything stalls and we register req_sel_n to be able to take back
      // from where we stalled. To stall we have to avoid going to the next req, but still
      // allow the previous one to end
        if (req_sel_n != req_sel_q) begin
          r_cnt <= r_cnt;
          req_sel_q <= DBG;
          resp_sel_q <= req_sel_q;
          req_sel_n_reg <= req_sel_n;
        end else begin
          req_sel_q <= req_sel_q;
          resp_sel_q <= req_sel_q;
        end
      end
    end
  end

  // Ideally we should switch the requests the cycle after grant arrives
  // and the responses the cycle after the r_valid arrives.
  // So we distinguish between a request mux and a response mux.

  // Request select signal
  always_comb begin
    req_sel_n = req_sel_q;
  
    case (req_sel_q)
      A:  begin   
          if (tcdm_muxed[2'b00].req & tcdm_muxed[2'b00].gnt) begin
            req_sel_n = B;
          end
        end
      B:  begin
          if (tcdm_muxed[2'b01].req & tcdm_muxed[2'b01].gnt) begin
            if (ctrl_i.fsm_ctrl.simple_mul) begin
              req_sel_n = D;
            end else begin
              if (r_cnt != ctrl_i.fsm_ctrl.len) begin
                req_sel_n = A; 
              end else begin
                req_sel_n = C;
              end
            end
          end
        end
      C:  begin
          if (tcdm_muxed[2'b10].req & tcdm_muxed[2'b10].gnt) begin
            req_sel_n = D;
          end
        end
      D:  begin   
          if (tcdm_muxed[2'b11].req & tcdm_muxed[2'b11].gnt) begin
            req_sel_n = A;
          end
        end
      DBG:begin
          	req_sel_n = req_sel_n_reg;
        	end
      default : req_sel_n = req_sel_q;
    endcase

  end

//-------------------------------------------------
// TCDM MUX
//-------------------------------------------------

always_comb begin

  // Default assignments
  tcdm[0].req  = '0;
  tcdm[0].add  = '0;
  tcdm[0].wen  = '0;
  tcdm[0].be   = '0;
  tcdm[0].data = '0;
  tcdm_muxed[3'b000].gnt     = '0;
  tcdm_muxed[3'b000].r_valid = '0;
  tcdm_muxed[3'b000].r_data  = '0;
  tcdm_muxed[3'b001].gnt     = '0;
  tcdm_muxed[3'b001].r_valid = '0;
  tcdm_muxed[3'b001].r_data  = '0;
  tcdm_muxed[3'b010].gnt     = '0;
  tcdm_muxed[3'b010].r_valid = '0;
  tcdm_muxed[3'b010].r_data  = '0;
  tcdm_muxed[3'b011].gnt     = '0;
  tcdm_muxed[3'b011].r_valid = '0;
  tcdm_muxed[3'b011].r_data  = '0;
  tcdm_muxed[3'b100].gnt     = '0;
  tcdm_muxed[3'b100].r_valid = '0;
  tcdm_muxed[3'b100].r_data  = '0;

  case (req_sel_q)
    A:  begin
        tcdm[0].req  = tcdm_muxed[3'b000].req;
        tcdm[0].add  = tcdm_muxed[3'b000].add;
        tcdm[0].wen  = tcdm_muxed[3'b000].wen;
        tcdm[0].be   = tcdm_muxed[3'b000].be;
        tcdm[0].data = tcdm_muxed[3'b000].data;

        tcdm_muxed[3'b000].gnt     = tcdm[0].gnt;
      end
    B:  begin
        tcdm[0].req  = tcdm_muxed[3'b001].req;
        tcdm[0].add  = tcdm_muxed[3'b001].add;
        tcdm[0].wen  = tcdm_muxed[3'b001].wen;
        tcdm[0].be   = tcdm_muxed[3'b001].be;
        tcdm[0].data = tcdm_muxed[3'b001].data;

        tcdm_muxed[3'b001].gnt     = tcdm[0].gnt;
      end
    C:  begin
        tcdm[0].req  = tcdm_muxed[3'b010].req;
        tcdm[0].add  = tcdm_muxed[3'b010].add;
        tcdm[0].wen  = tcdm_muxed[3'b010].wen;
        tcdm[0].be   = tcdm_muxed[3'b010].be;
        tcdm[0].data = tcdm_muxed[3'b010].data;

        tcdm_muxed[3'b010].gnt     = tcdm[0].gnt;
      end
    D:  begin
        tcdm[0].req  = tcdm_muxed[3'b011].req;
        tcdm[0].add  = tcdm_muxed[3'b011].add;
        tcdm[0].wen  = tcdm_muxed[3'b011].wen;
        tcdm[0].be   = tcdm_muxed[3'b011].be;
        tcdm[0].data = tcdm_muxed[3'b011].data;

        tcdm_muxed[3'b011].gnt     = tcdm[0].gnt;
      end
    DBG: begin
        tcdm[0].req  = tcdm_muxed[3'b100].req;
        tcdm[0].add  = tcdm_muxed[3'b100].add;
        tcdm[0].wen  = tcdm_muxed[3'b100].wen;
        tcdm[0].be   = tcdm_muxed[3'b100].be;
        tcdm[0].data = tcdm_muxed[3'b100].data;

        tcdm_muxed[3'b100].gnt     = tcdm[0].gnt;
      end
    default : ;
  endcase

  case (resp_sel_q)
    A:  begin
        tcdm_muxed[3'b000].r_valid = tcdm[0].r_valid;
        tcdm_muxed[3'b000].r_data  = tcdm[0].r_data;
      end
    B:  begin
        tcdm_muxed[3'b001].r_valid = tcdm[0].r_valid;
        tcdm_muxed[3'b001].r_data  = tcdm[0].r_data;
      end
    C:  begin
        tcdm_muxed[3'b010].r_valid = tcdm[0].r_valid;
        tcdm_muxed[3'b010].r_data  = tcdm[0].r_data;
      end
    D:  begin
        tcdm_muxed[3'b011].r_valid = tcdm[0].r_valid;
        tcdm_muxed[3'b011].r_data  = tcdm[0].r_data;
      end
    DBG:  begin
        tcdm_muxed[3'b100].r_valid = tcdm[0].r_valid;
        tcdm_muxed[3'b100].r_data  = tcdm[0].r_data;
      end
    default : ;
  endcase

end


  // datapath-side FIFOs
  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2  ),
    .LATCH_FIFO( 0  )
  ) i_a_fifo (
    .clk_i   ( clk_i          ),
    .rst_ni  ( rst_ni         ),
    .clear_i ( clear_i        ),
    .push_i  ( a_prefifo.sink ),
    .pop_o   ( a_o            ),
    .flags_o (                )
  );

  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2  ),
    .LATCH_FIFO( 0  )
  ) i_b_fifo (
    .clk_i   ( clk_i          ),
    .rst_ni  ( rst_ni         ),
    .clear_i ( clear_i        ),
    .push_i  ( b_prefifo.sink ),
    .pop_o   ( b_o            ),
    .flags_o (                )
  );

  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2  ),
    .LATCH_FIFO( 0  )
  ) i_c_fifo (
    .clk_i   ( clk_i          ),
    .rst_ni  ( rst_ni         ),
    .clear_i ( clear_i        ),
    .push_i  ( c_prefifo.sink ),
    .pop_o   ( c_o            ),
    .flags_o (                )
  );

  hwpe_stream_fifo #(
    .DATA_WIDTH( DATA_WIDTH ),
    .FIFO_DEPTH( 2  ),
    .LATCH_FIFO( 0  )
  ) i_d_fifo (
    .clk_i   ( clk_i             ),
    .rst_ni  ( rst_ni            ),
    .clear_i ( clear_i           ),
    .push_i  ( d_i               ),
    .pop_o   ( d_postfifo.source ),
    .flags_o (                   )
  );

endmodule // mac_streamer
