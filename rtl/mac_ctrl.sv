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

  // Debug signals
  logic dbg_active;
  logic r_toggle_dbg;
  logic r_step_dbg;
  logic r_hreq_dbg;

  logic [31:0] r_haddr_dbg_q;
  logic [31:0] r_haddr_dbg_n;
  logic r_hwen_dbg_q;
  logic r_hwen_dbg_n;
  logic [31:0] r_hwdata_dbg_q;
  logic [31:0] r_hwdata_dbg_n;
  logic [31:0] r_hrdata_dbg_q;
  logic [31:0] r_hrdata_dbg_n;

/////////////////////////////////////////////////////////////////
// MUX periph requests between hwpe_ctrl_slave and debug system
/////////////////////////////////////////////////////////////////
  hwpe_ctrl_intf_periph periph_2_slave(.clk(clk_i));
  logic periph_dest_slave, periph_dest_slave_delayed;

  logic         periph_rvalid;
  logic [31:0]  periph_rdata;
  logic         periph_handshaked;

// MUX
  always_comb begin
    periph_2_slave.req   = periph_dest_slave ? periph.req : '0;
    periph_2_slave.add   = periph_dest_slave ? periph.add : '0;
    periph_2_slave.wen   = periph_dest_slave ? periph.wen : '0;
    periph_2_slave.be    = periph_dest_slave ? periph.be : '0;
    periph_2_slave.data  = periph_dest_slave ? periph.data : '0;
    periph_2_slave.id    = periph_dest_slave ? periph.id : '0;
    periph.gnt           = periph_dest_slave ? periph_2_slave.gnt : 1'b1;
    periph.r_data        = periph_dest_slave_delayed ? periph_2_slave.r_data : periph_rdata;
    periph.r_valid       = periph_dest_slave_delayed ? periph_2_slave.r_valid : periph_rvalid;
    periph.r_id          = periph_dest_slave_delayed ? periph_2_slave.r_id : '0;  
  end

  // MUX select
  // Addresses after 0xA000_1000 are debug register read-only values.
  assign periph_dest_slave = (periph.add[12]==1'b1) ? 1'b0 : 1'b1;

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

  // DEBUG read-only signals
  always_comb begin
    periph_rdata = '0;

    if (periph_rvalid) begin
      case (periph.add[7:2])

        6'b000000: periph_rdata = fsm_flags.state;
        6'b000001: periph_rdata = flags_engine_i.cnt;
        6'b000010: periph_rdata = flags_streamer_i.a_addr;
        6'b000011: periph_rdata = flags_streamer_i.b_addr;
        6'b000100: periph_rdata = flags_streamer_i.c_addr;
        6'b000101: periph_rdata = flags_streamer_i.d_addr;
        6'b000110: periph_rdata = reg_file.hwpe_params[MAC_REG_NB_ITER];
        6'b000111: periph_rdata = reg_file.hwpe_params[MAC_REG_LEN_ITER];
        6'b001000: periph_rdata = reg_file.hwpe_params[MAC_REG_SHIFT_SIMPLEMUL];
        6'b001001: periph_rdata = reg_file.hwpe_params[MAC_REG_SHIFT_VECTSTRIDE];
        6'b001010: periph_rdata = started;
        6'b001011: periph_rdata = done;
        6'b001100: periph_rdata = flags_streamer_i.a_source_flags.ready_start;
        6'b001101: periph_rdata = flags_streamer_i.b_source_flags.ready_start;
        6'b001110: periph_rdata = flags_streamer_i.c_source_flags.ready_start;
        6'b001111: periph_rdata = flags_streamer_i.d_sink_flags.ready_start;
        6'b010000: periph_rdata = flags_engine_i.started;
        6'b010001: periph_rdata = flags_streamer_i.a_source_flags.state;
        6'b010010: periph_rdata = flags_streamer_i.b_source_flags.state;
        6'b010011: periph_rdata = flags_streamer_i.c_source_flags.state;
        6'b010100: periph_rdata = flags_streamer_i.d_sink_flags.state;
        6'b010101: periph_rdata = ucode_flags.offs[MAC_UCODE_A_OFFS];
        6'b010110: periph_rdata = ucode_flags.offs[MAC_UCODE_B_OFFS];
        6'b010111: periph_rdata = ucode_flags.offs[MAC_UCODE_C_OFFS];
        6'b011000: periph_rdata = ucode_flags.offs[MAC_UCODE_D_OFFS];

        6'b011001: periph_rdata = flags_engine_i.a;
        6'b011010: periph_rdata = flags_engine_i.b;
        6'b011011: periph_rdata = flags_engine_i.c;
        6'b011100: periph_rdata = flags_engine_i.d;
        6'b011101: periph_rdata = flags_engine_i.mult[31:0];
        6'b011110: periph_rdata = flags_engine_i.mult[63:32];
        6'b011111: periph_rdata = flags_engine_i.r_acc[31:0];
        6'b100000: periph_rdata = flags_engine_i.r_acc[63:32];
        6'b100001: periph_rdata = flags_engine_i.mult_valid;
        6'b100010: periph_rdata = flags_engine_i.mult_ready;
        6'b100011: periph_rdata = flags_engine_i.d_valid;
        6'b100100: periph_rdata = flags_engine_i.acc_done;

        6'b100101: periph_rdata = dbg_active;
        6'b100110: periph_rdata = r_hrdata_dbg_q;

        default : periph_rdata = '0;

      endcase
    end
  end


  assign ctrl_streamer_o.dbg_active = dbg_active;
  assign ctrl_streamer_o.dbg_step = r_step_dbg;

  // DEBUG write registers (from address 0xA000_1100)
  always_comb begin
    r_toggle_dbg = '0;
    r_step_dbg = '0;
    r_hreq_dbg = '0;
    r_haddr_dbg_n = r_haddr_dbg_q;
    r_hwen_dbg_n = r_hwen_dbg_q;
    r_hwdata_dbg_n = r_hwdata_dbg_q;

    if (periph.req & (~periph_dest_slave) & ~periph.wen) begin
      if (periph.add[8] == 1'b1) begin
        case (periph.add[5:2])

          3'b000: r_toggle_dbg = periph.data;
          3'b001: r_step_dbg = periph.data;
          3'b010: r_haddr_dbg_n = periph.data;
          3'b011: r_hwen_dbg_n = periph.data;
          3'b100: r_hwdata_dbg_n = periph.data;
          3'b101: r_hreq_dbg = periph.data;

          default: ;

        endcase
      end
    end
  end

  assign ctrl_streamer_o.dbg_hreq = r_hreq_dbg;
  assign ctrl_streamer_o.dbg_haddr = r_haddr_dbg_q;
  assign ctrl_streamer_o.dbg_hwen = r_hwen_dbg_q;
  assign ctrl_streamer_o.dbg_hwdata = r_hwdata_dbg_q;
  assign r_hrdata_dbg_n = flags_streamer_i.dbg_hrvalid ? flags_streamer_i.dbg_hrdata : r_hrdata_dbg_q;

  // Debug AHB test registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      r_haddr_dbg_q <= 0;
      r_hwen_dbg_q <= 0;
      r_hwdata_dbg_q <= 0;
      r_hrdata_dbg_q <= 0;
    end else begin
      r_haddr_dbg_q <= r_haddr_dbg_n;
      r_hwen_dbg_q <= r_hwen_dbg_n;
      r_hwdata_dbg_q <= r_hwdata_dbg_n;
      r_hrdata_dbg_q <= r_hrdata_dbg_n;
    end
  end

  // DEBUG MODE REGISTER
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      dbg_active <= 1'b0;
    end else begin
      if (r_toggle_dbg) begin
        dbg_active <= ~dbg_active;
      end else begin
        dbg_active <= dbg_active;
      end
    end
  end

  // STARTED REGISTER
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

  // DONE REGISTER
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
    .ctrl_streamer_o  ( ctrl_streamer_o.fsm_ctrl    ),
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
