/* 
 * mac_ctrl.sv
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
import hwpe_ctrl_package::*;

module mac_ctrl
#(
  parameter int unsigned N_CORES         = 2,
  parameter int unsigned N_CONTEXT       = 2,
  parameter int unsigned N_IO_REGS       = 16,
  parameter int unsigned ID              = 10,
  parameter int unsigned UCODE_HARDWIRED = 0
)
(
  // global signals
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  test_mode_i,
  output logic                                  clear_o,
  // events
  output logic [N_CORES-1:0][REGFILE_N_EVT-1:0] evt_o,
  // ctrl & flags
  output ctrl_streamer_t                        ctrl_streamer_o,
  input  flags_streamer_t                       flags_streamer_i,
  output ctrl_engine_t                          ctrl_engine_o,
  input  flags_engine_t                         flags_engine_i,
  // periph slave port
  hwpe_ctrl_intf_periph.slave                   periph
);

  ctrl_slave_t   slave_ctrl;
  flags_slave_t  slave_flags;
  ctrl_regfile_t reg_file;

  logic unsigned [31:0] static_reg_nb_iter;
  logic unsigned [31:0] static_reg_len_iter;
  logic unsigned [31:0] static_reg_vectstride;
  logic unsigned [31:0] static_reg_onestride;
  logic unsigned [15:0] static_reg_shift;
  logic static_reg_simplemul;

  logic [223:0]  ucode_flat;
  ucode_t ucode;
  ctrl_ucode_t   ucode_ctrl;
  flags_ucode_t  ucode_flags;
  logic [11:0][31:0] ucode_registers_read;

  ctrl_fsm_t fsm_ctrl;
  flags_fsm_t fsm_flags;

  logic started, done;

  // logic [31:0] dbg_regs [0:31];
  // logic [31:0] dbg_regs_next [0:31];

  // genvar i;
  // generate
  //   for (i=0; i<32; i++) begin

  //     always_ff @(posedge clk_i or negedge rst_ni) begin
  //       if(~rst_ni) begin
  //         dbg_regs[i] <= 0;
  //       end else begin
  //         dbg_regs[i] <= dbg_regs_next[i];
  //       end
  //     end  

  //   end
  // endgenerate

  hwpe_ctrl_intf_periph periph_2_slave(.clk(clk_i));
  logic periph_dest_slave, periph_dest_slave_delayed;

  logic         periph_rvalid;
  logic [31:0]  periph_rdata;
  logic         periph_handshaked;

  always_comb begin
    periph_2_slave.req   = periph.req;
    periph_2_slave.add   = periph.add;
    periph_2_slave.wen   = periph.wen;
    periph_2_slave.be    = periph.be;
    periph_2_slave.data  = periph.data;
    periph_2_slave.id    = periph.id;
    periph.gnt           = periph_dest_slave ? periph_2_slave.gnt : 1'b1;
    periph.r_data        = periph_dest_slave_delayed ? periph_2_slave.r_data : periph_rdata;
    periph.r_valid       = periph_dest_slave_delayed ? periph_2_slave.r_valid : periph_rvalid;
    periph.r_id          = periph_dest_slave_delayed ? periph_2_slave.r_id : '0;  
  end

  // Addresses after 0xA000_1000 are debug values.
  assign periph_dest_slave = (periph.add[12]==1'b1) ? 1'b0 : 1'b1;

  assign periph_gnt = 1'b1;

  always_comb begin
    periph_handshaked = 1'b0;
    if (periph.req & (~periph_dest_slave)) begin
      periph_handshaked = 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      periph_rvalid <= 0;
      periph_dest_slave_delayed <= 0;
    end else begin
      periph_rvalid <= periph_handshaked;
      periph_dest_slave_delayed <= periph_dest_slave; 
    end
  end

  // MUX dbg signals
  always_comb begin
    periph_rdata = '0;

    if (periph_rvalid) begin
      case (periph.add[5:2])

        4'b0000: periph_rdata = fsm_flags.state;
        4'b0001: periph_rdata = flags_engine_i.cnt;
        4'b0010: periph_rdata = flags_streamer_i.a_addr;
        4'b0011: periph_rdata = flags_streamer_i.b_addr;
        4'b0100: periph_rdata = flags_streamer_i.c_addr;
        4'b0101: periph_rdata = flags_streamer_i.d_addr;
        4'b0110: periph_rdata = reg_file.hwpe_params[MAC_REG_NB_ITER];
        4'b0111: periph_rdata = reg_file.hwpe_params[MAC_REG_LEN_ITER];
        4'b1000: periph_rdata = reg_file.hwpe_params[MAC_REG_SHIFT_SIMPLEMUL];
        4'b1001: periph_rdata = reg_file.hwpe_params[MAC_REG_SHIFT_VECTSTRIDE];
        4'b1010: periph_rdata = started;
        4'b1011: periph_rdata = done;
        4'b1100: periph_rdata = flags_streamer_i.a_source_flags.ready_start;
        4'b1101: periph_rdata = flags_streamer_i.b_source_flags.ready_start;
        4'b1110: periph_rdata = flags_streamer_i.c_source_flags.ready_start;
        4'b1111: periph_rdata = flags_streamer_i.d_sink_flags.ready_start;

        default : periph_rdata = '0;

      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      started <= 0;
    end else begin
      if (slave_flags.start) begin
        started <= 1;
      end else begin
        started <= started;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      done <= 0;
    end else begin
      if (slave_ctrl.done) begin
        done <= 1;
      end else begin
        done <= done;
      end
    end
  end


  /* Peripheral slave & register file */
  hwpe_ctrl_slave #(
    .N_CORES        ( N_CORES               ),
    .N_CONTEXT      ( N_CONTEXT             ),
    .N_IO_REGS      ( N_IO_REGS             ),
    .N_GENERIC_REGS ( (1-UCODE_HARDWIRED)*8 ),
    .ID_WIDTH       ( ID                    )
  ) i_slave (
    .clk_i    ( clk_i       ),
    .rst_ni   ( rst_ni      ),
    .clear_o  ( clear_o     ),
    .cfg      ( periph_2_slave ),
    .ctrl_i   ( slave_ctrl  ),
    .flags_o  ( slave_flags ),
    .reg_file ( reg_file    )
  );
  assign evt_o = slave_flags.evt;

  /* Direct register file mappings */
  assign static_reg_nb_iter    = reg_file.hwpe_params[MAC_REG_NB_ITER]  + 1;
  assign static_reg_len_iter   = reg_file.hwpe_params[MAC_REG_LEN_ITER] + 1;
  assign static_reg_shift      = reg_file.hwpe_params[MAC_REG_SHIFT_SIMPLEMUL][31:16];
  assign static_reg_simplemul  = reg_file.hwpe_params[MAC_REG_SHIFT_SIMPLEMUL][0];
  assign static_reg_vectstride = reg_file.hwpe_params[MAC_REG_SHIFT_VECTSTRIDE];
  assign static_reg_onestride  = 4;

  /* Microcode processor */
  generate
    if(UCODE_HARDWIRED != 0) begin
      // equivalent to the microcode in ucode/mac_code.yml
      assign ucode_flat = 224'h0000000000040000000000000000000000000000000008cd11a12c05;
    end
    else begin
      // the microcode is stored in registers independent of context (job)
      assign ucode_flat = reg_file.generic_params[6:0];
    end
  endgenerate
  assign ucode = { 
    // loops & bytecode
    ucode_flat,
    // ranges
    12'b0,
    12'b0,
    12'b0,
    12'b0,
    12'b0,
    static_reg_nb_iter[11:0]
  };
  assign ucode_registers_read[MAC_UCODE_MNEM_NBITER]     = static_reg_nb_iter;
  assign ucode_registers_read[MAC_UCODE_MNEM_ITERSTRIDE] = static_reg_vectstride;
  assign ucode_registers_read[MAC_UCODE_MNEM_ONESTRIDE]  = static_reg_onestride;
  assign ucode_registers_read[11:3] = '0;
  hwpe_ctrl_ucode #(
    .NB_LOOPS  ( 1  ),
    .NB_REG    ( 4  ),
    .NB_RO_REG ( 12 )
  ) i_ucode (
    .clk_i            ( clk_i                ),
    .rst_ni           ( rst_ni               ),
    .test_mode_i      ( test_mode_i          ),
    .clear_i          ( clear_o              ),
    .ctrl_i           ( ucode_ctrl           ),
    .flags_o          ( ucode_flags          ),
    .ucode_i          ( ucode                ),
    .registers_read_i ( ucode_registers_read )
  );

  /* Main FSM */
  mac_fsm i_fsm (
    .clk_i            ( clk_i              ),
    .rst_ni           ( rst_ni             ),
    .test_mode_i      ( test_mode_i        ),
    .clear_i          ( clear_o            ),
    .ctrl_streamer_o  ( ctrl_streamer_o    ),
    .flags_streamer_i ( flags_streamer_i   ),
    .ctrl_engine_o    ( ctrl_engine_o      ),
    .flags_engine_i   ( flags_engine_i     ),
    .ctrl_ucode_o     ( ucode_ctrl         ),
    .flags_ucode_i    ( ucode_flags        ),
    .ctrl_slave_o     ( slave_ctrl         ),
    .flags_slave_i    ( slave_flags        ),
    .reg_file_i       ( reg_file           ),
    .ctrl_i           ( fsm_ctrl           ),
    .flags_o          ( fsm_flags          )
  );
  always_comb
  begin
    fsm_ctrl.simple_mul = static_reg_simplemul;
    fsm_ctrl.shift      = static_reg_shift[$clog2(32)-1:0];
    fsm_ctrl.len        = static_reg_len_iter[$clog2(MAC_CNT_LEN):0];
  end

endmodule // mac_ctrl
