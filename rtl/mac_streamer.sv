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

  logic a_tcdm_fifo_ready, b_tcdm_fifo_ready, c_tcdm_fifo_ready;

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

  hwpe_stream_intf_tcdm tcdm_fifo [3:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_fifo_0 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_fifo_1 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_fifo_2 [0:0] (
    .clk ( clk_i )
  );
  hwpe_stream_intf_tcdm tcdm_fifo_3 [0:0] (
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
    .tcdm               ( tcdm_fifo_0            ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( a_prefifo.source/*a_o*/       ),
    .ctrl_i             ( ctrl_i.a_source_ctrl   ),
    .flags_o            ( flags_o.a_source_flags ),
    .tcdm_fifo_ready_o  ( a_tcdm_fifo_ready      )
  );

  hwpe_stream_source #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .DECOUPLED  ( 1  )
  ) i_b_source (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .tcdm               ( tcdm_fifo_1            ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( b_prefifo.source/*b_o*/       ),
    .ctrl_i             ( ctrl_i.b_source_ctrl   ),
    .flags_o            ( flags_o.b_source_flags ),
    .tcdm_fifo_ready_o  ( b_tcdm_fifo_ready      )
  );

  hwpe_stream_source #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .DECOUPLED  ( 1  )
  ) i_c_source (
    .clk_i              ( clk_i                  ),
    .rst_ni             ( rst_ni                 ),
    .test_mode_i        ( test_mode_i            ),
    .clear_i            ( clear_i                ),
    .tcdm               ( tcdm_fifo_2            ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream             ( c_prefifo.source/*c_o*/       ),
    .ctrl_i             ( ctrl_i.c_source_ctrl   ),
    .flags_o            ( flags_o.c_source_flags ),
    .tcdm_fifo_ready_o  ( c_tcdm_fifo_ready      )
  );

  hwpe_stream_sink #(
    .DATA_WIDTH ( DATA_WIDTH ),
    .USE_TCDM_FIFOS ( 0 )
  ) i_d_sink (
    .clk_i       ( clk_i                ),
    .rst_ni      ( rst_ni               ),
    .test_mode_i ( test_mode_i          ),
    .clear_i     ( clear_i              ),
    .tcdm        ( tcdm_fifo_3          ), // this syntax is necessary for Verilator as hwpe_stream_source expects an array of interfaces
    .stream      ( d_postfifo.sink/*d_i*/      ),
    .ctrl_i      ( ctrl_i.d_sink_ctrl   ),
    .flags_o     ( flags_o.d_sink_flags )
  );


  // // TCDM-side FIFOs
  // hwpe_stream_tcdm_fifo_load #(
  //   .FIFO_DEPTH ( 4 )
  // ) i_a_tcdm_fifo_load (
  //   .clk_i       ( clk_i             ),
  //   .rst_ni      ( rst_ni            ),
  //   .clear_i     ( clear_i           ),
  //   .flags_o     (                   ),
  //   .ready_i     ( a_tcdm_fifo_ready ),
  //   .tcdm_slave  ( tcdm_fifo_0[0]    ),
  //   .tcdm_master ( tcdm_fifo[0]      )
  // );

  // hwpe_stream_tcdm_fifo_load #(
  //   .FIFO_DEPTH ( 4 )
  // ) i_b_tcdm_fifo_load (
  //   .clk_i       ( clk_i             ),
  //   .rst_ni      ( rst_ni            ),
  //   .clear_i     ( clear_i           ),
  //   .flags_o     (                   ),
  //   .ready_i     ( b_tcdm_fifo_ready ),
  //   .tcdm_slave  ( tcdm_fifo_1[0]    ),
  //   .tcdm_master ( tcdm_fifo[1]      )
  // );

  // hwpe_stream_tcdm_fifo_load #(
  //   .FIFO_DEPTH ( 4 )
  // ) i_c_tcdm_fifo_load (
  //   .clk_i       ( clk_i             ),
  //   .rst_ni      ( rst_ni            ),
  //   .clear_i     ( clear_i           ),
  //   .flags_o     (                   ),
  //   .ready_i     ( c_tcdm_fifo_ready ),
  //   .tcdm_slave  ( tcdm_fifo_2[0]    ),
  //   .tcdm_master ( tcdm_fifo[2]      )
  // );

  // hwpe_stream_tcdm_fifo_store #(
  //   .FIFO_DEPTH ( 4 )
  // ) i_d_tcdm_fifo_store (
  //   .clk_i       ( clk_i          ),
  //   .rst_ni      ( rst_ni         ),
  //   .clear_i     ( clear_i        ),
  //   .flags_o     (                ),
  //   .tcdm_slave  ( tcdm_fifo_3[0] ),
  //   .tcdm_master ( tcdm_fifo[3]   )
  // );

always_comb begin
  tcdm_fifo[0].req        = tcdm_fifo_0[0].req;
  tcdm_fifo[0].add        = tcdm_fifo_0[0].add;
  tcdm_fifo[0].wen        = tcdm_fifo_0[0].wen;
  tcdm_fifo[0].be         = tcdm_fifo_0[0].be;
  tcdm_fifo[0].data       = tcdm_fifo_0[0].data;
  tcdm_fifo_0[0].gnt      = tcdm_fifo[0].gnt;
  tcdm_fifo_0[0].r_data   = tcdm_fifo[0].r_data;
  tcdm_fifo_0[0].r_valid  = tcdm_fifo[0].r_valid;

  tcdm_fifo[1].req        = tcdm_fifo_1[0].req;
  tcdm_fifo[1].add        = tcdm_fifo_1[0].add;
  tcdm_fifo[1].wen        = tcdm_fifo_1[0].wen;
  tcdm_fifo[1].be         = tcdm_fifo_1[0].be;
  tcdm_fifo[1].data       = tcdm_fifo_1[0].data;
  tcdm_fifo_1[0].gnt      = tcdm_fifo[1].gnt;
  tcdm_fifo_1[0].r_data   = tcdm_fifo[1].r_data;
  tcdm_fifo_1[0].r_valid  = tcdm_fifo[1].r_valid;

  tcdm_fifo[2].req        = tcdm_fifo_2[0].req;
  tcdm_fifo[2].add        = tcdm_fifo_2[0].add;
  tcdm_fifo[2].wen        = tcdm_fifo_2[0].wen;
  tcdm_fifo[2].be         = tcdm_fifo_2[0].be;
  tcdm_fifo[2].data       = tcdm_fifo_2[0].data;
  tcdm_fifo_2[0].gnt      = tcdm_fifo[2].gnt;
  tcdm_fifo_2[0].r_data   = tcdm_fifo[2].r_data;
  tcdm_fifo_2[0].r_valid  = tcdm_fifo[2].r_valid;

  tcdm_fifo[3].req        = tcdm_fifo_3[0].req;
  tcdm_fifo[3].add        = tcdm_fifo_3[0].add;
  tcdm_fifo[3].wen        = tcdm_fifo_3[0].wen;
  tcdm_fifo[3].be         = tcdm_fifo_3[0].be;
  tcdm_fifo[3].data       = tcdm_fifo_3[0].data;
  tcdm_fifo_3[0].gnt      = tcdm_fifo[3].gnt;
  tcdm_fifo_3[0].r_data   = tcdm_fifo[3].r_data;
  tcdm_fifo_3[0].r_valid  = tcdm_fifo[3].r_valid;
end


	typedef enum {A=0, B=1, C=2, D=3} tcdm_id;
	tcdm_id req_sel_q, req_sel_n, resp_sel_q;
	logic [$clog2(MAC_CNT_LEN):0] cnt, r_cnt;

  always_comb begin
    cnt = r_cnt + 1'b1;
    if ( r_cnt == ctrl_i.len ) begin
      cnt = '0;
    end
  end

	always_ff @(posedge clk_i or negedge rst_ni)
	begin : main_fsm_seq
		if(~rst_ni) begin
			r_cnt <= '0;
			req_sel_q <= A;
			resp_sel_q <= A;
		end
		else if(clear_i) begin
			r_cnt <= '0;
			req_sel_q <= A;
			resp_sel_q <= A;
		end
		else begin
      if ( (req_sel_q == A && req_sel_n == B) | (r_cnt == ctrl_i.len && req_sel_q == D) ) begin
			  r_cnt <= cnt;
			end else begin
        r_cnt <= r_cnt;
      end
      req_sel_q <= req_sel_n;
			// Response select signal is simply the delayed request signal
			// since r_valid is guaranteed the cycle after the grant
			resp_sel_q <= req_sel_q;
		end
	end

	// Ideally we should switch the requests the cycle after grant arrives
	// and the responses the cycle after the r_valid arrives.
	// So we distinguish between a request mux and a response mux.

	// Request select signal
	always_comb begin
		req_sel_n = req_sel_q;
	
		case (req_sel_q)
			A:	begin 	
					if (tcdm_fifo[2'b00].gnt) begin
						req_sel_n = B;
					end
				end
			B: 	begin
					if (tcdm_fifo[2'b01].gnt) begin
						if (ctrl_i.simple_mul) begin
							req_sel_n = D;
						end else begin
              if (r_cnt != ctrl_i.len) begin
                req_sel_n = A; 
							end else begin
                req_sel_n = C;
						  end
            end
					end
				end
			C:	begin
					if (tcdm_fifo[2'b10].gnt) begin
						req_sel_n = D;
					end
				end
			D:	begin 	
					if (tcdm_fifo[2'b11].gnt) begin
						req_sel_n = A;
					end
				end
			default : req_sel_n = req_sel_q;
		endcase

	end

	// Response select signal
	// always_comb begin
	// 	resp_sel_n = resp_sel_q;
	
	// 	case (resp_sel_q)
	// 		A:	begin 	
	// 				if (tcdm_fifo[2'b00].r_valid) begin
	// 					resp_sel_n = B;
	// 				end
	// 			end
	// 		B: 	begin
	// 				if (tcdm_fifo[2'b01].r_valid) begin
	// 					if (ctrl_i.simple_mul) begin
	// 						resp_sel_n = D;
	// 					end else begin 
	// 						resp_sel_n = C;
	// 					end
	// 				end
	// 			end
	// 		C:	begin
	// 				if (tcdm_fifo[2'b10].r_valid) begin
	// 					resp_sel_n = D;
	// 				end
	// 			end
	// 		D:	begin 	
	// 				if (tcdm_fifo[2'b11].r_valid) begin
	// 					resp_sel_n = A;
	// 				end
	// 			end
	// 		default : resp_sel_n = resp_sel_q;
	// 	endcase

	// end

//-------------------------------------------------
// TCDM MUX
//-------------------------------------------------

always_comb begin

	// Default assignments
	tcdm[0].req  = 'z;
	tcdm[0].add  = 'z;
	tcdm[0].wen  = 'z;
	tcdm[0].be   = 'z;
	tcdm[0].data = 'z;
	tcdm_fifo[2'b00].gnt     = '0;
	tcdm_fifo[2'b00].r_valid = '0;
	tcdm_fifo[2'b00].r_data  = 'z;
	tcdm_fifo[2'b01].gnt     = '0;
	tcdm_fifo[2'b01].r_valid = '0;
	tcdm_fifo[2'b01].r_data  = 'z;
	tcdm_fifo[2'b10].gnt     = '0;
	tcdm_fifo[2'b10].r_valid = '0;
	tcdm_fifo[2'b10].r_data  = 'z;
	tcdm_fifo[2'b11].gnt     = '0;
	tcdm_fifo[2'b11].r_valid = '0;
	tcdm_fifo[2'b11].r_data  = 'z;

	case (req_sel_q)
		A:	begin
				tcdm[0].req  = tcdm_fifo[2'b00].req;
				tcdm[0].add  = tcdm_fifo[2'b00].add;
				tcdm[0].wen  = tcdm_fifo[2'b00].wen;
				tcdm[0].be   = tcdm_fifo[2'b00].be;
				tcdm[0].data = tcdm_fifo[2'b00].data;

				tcdm_fifo[2'b00].gnt     = tcdm[0].gnt;
				// tcdm_fifo[2'b00].r_valid = tcdm[0].r_valid;
				// tcdm_fifo[2'b00].r_data  = tcdm[0].r_data;
			end
		B:	begin
				tcdm[0].req  = tcdm_fifo[2'b01].req;
				tcdm[0].add  = tcdm_fifo[2'b01].add;
				tcdm[0].wen  = tcdm_fifo[2'b01].wen;
				tcdm[0].be   = tcdm_fifo[2'b01].be;
				tcdm[0].data = tcdm_fifo[2'b01].data;

				tcdm_fifo[2'b01].gnt     = tcdm[0].gnt;
				// tcdm_fifo[2'b01].r_valid = tcdm[0].r_valid;
				// tcdm_fifo[2'b01].r_data  = tcdm[0].r_data;
			end
		C:	begin
				tcdm[0].req  = tcdm_fifo[2'b10].req;
				tcdm[0].add  = tcdm_fifo[2'b10].add;
				tcdm[0].wen  = tcdm_fifo[2'b10].wen;
				tcdm[0].be   = tcdm_fifo[2'b10].be;
				tcdm[0].data = tcdm_fifo[2'b10].data;

				tcdm_fifo[2'b10].gnt     = tcdm[0].gnt;
				// tcdm_fifo[2'b10].r_valid = tcdm[0].r_valid;
				// tcdm_fifo[2'b10].r_data  = tcdm[0].r_data;
			end
		D:	begin
				tcdm[0].req  = tcdm_fifo[2'b11].req;
				tcdm[0].add  = tcdm_fifo[2'b11].add;
				tcdm[0].wen  = tcdm_fifo[2'b11].wen;
				tcdm[0].be   = tcdm_fifo[2'b11].be;
				tcdm[0].data = tcdm_fifo[2'b11].data;

				tcdm_fifo[2'b11].gnt     = tcdm[0].gnt;
				// tcdm_fifo[2'b11].r_valid = tcdm[0].r_valid;
				// tcdm_fifo[2'b11].r_data  = tcdm[0].r_data;
			end
		default : ;
	endcase

	case (resp_sel_q)
		A:	begin
				// tcdm[0].req  = tcdm_fifo[2'b00].req;
				// tcdm[0].add  = tcdm_fifo[2'b00].add;
				// tcdm[0].wen  = tcdm_fifo[2'b00].wen;
				// tcdm[0].be   = tcdm_fifo[2'b00].be;
				// tcdm[0].data = tcdm_fifo[2'b00].data;

				// tcdm_fifo[2'b00].gnt     = tcdm[0].gnt;
				tcdm_fifo[2'b00].r_valid = tcdm[0].r_valid;
				tcdm_fifo[2'b00].r_data  = tcdm[0].r_data;
			end
		B:	begin
				// tcdm[0].req  = tcdm_fifo[2'b01].req;
				// tcdm[0].add  = tcdm_fifo[2'b01].add;
				// tcdm[0].wen  = tcdm_fifo[2'b01].wen;
				// tcdm[0].be   = tcdm_fifo[2'b01].be;
				// tcdm[0].data = tcdm_fifo[2'b01].data;

				// tcdm_fifo[2'b01].gnt     = tcdm[0].gnt;
				tcdm_fifo[2'b01].r_valid = tcdm[0].r_valid;
				tcdm_fifo[2'b01].r_data  = tcdm[0].r_data;
			end
		C:	begin
				// tcdm[0].req  = tcdm_fifo[2'b10].req;
				// tcdm[0].add  = tcdm_fifo[2'b10].add;
				// tcdm[0].wen  = tcdm_fifo[2'b10].wen;
				// tcdm[0].be   = tcdm_fifo[2'b10].be;
				// tcdm[0].data = tcdm_fifo[2'b10].data;

				// tcdm_fifo[2'b10].gnt     = tcdm[0].gnt;
				tcdm_fifo[2'b10].r_valid = tcdm[0].r_valid;
				tcdm_fifo[2'b10].r_data  = tcdm[0].r_data;
			end
		D:	begin
				// tcdm[0].req  = tcdm_fifo[2'b11].req;
				// tcdm[0].add  = tcdm_fifo[2'b11].add;
				// tcdm[0].wen  = tcdm_fifo[2'b11].wen;
				// tcdm[0].be   = tcdm_fifo[2'b11].be;
				// tcdm[0].data = tcdm_fifo[2'b11].data;

				// tcdm_fifo[2'b11].gnt     = tcdm[0].gnt;
				tcdm_fifo[2'b11].r_valid = tcdm[0].r_valid;
				tcdm_fifo[2'b11].r_data  = tcdm[0].r_data;
			end
		default : ;
	endcase

end

  // TCDM-side MUX: 4 FIFOs are MUXed on MP memory interfaces
  // hwpe_stream_tcdm_mux_static #(
  //   .NB_IN_CHAN   ( 4 )
  // ) i_tcdm_fifo_mux (
  //   .clk_i        ( clk_i         ),
  //   .rst_ni       ( rst_ni        ),
  //   .clear_i      ( clear_i       ),
  //   .sel_i        ( sel_q         ),
  //   .in           ( tcdm_fifo     ),
  //   .out          ( tcdm          )
  // );




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
