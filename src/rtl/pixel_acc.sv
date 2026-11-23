// ============================================================
//  pixel_acc - STUDENT MODULE
//
//  You are given direct read access to ibuf and write access
//  to obuf. Both are synchronous BRAMs with 1-cycle read latency.
//
//  Interface:
//    start  - pulsed high for 1 cycle to begin processing
//    finish - assert high for 1 cycle when all output words
//             have been written to obuf
//
//  ibuf read port:
//    Set ibuf_rd_en=1 and ibuf_rd_addr to the word you want.
//    ibuf_rd_data is valid on the NEXT clock cycle.
//
//  obuf write port:
//    Set obuf_wr_en=1, obuf_wr_addr, obuf_wr_data on the same cycle.
//    The word is written on the rising edge.
//
//  Parameters:
//    BUF_AW    - address width (ibuf/obuf have 2^BUF_AW words)
//    BUF_DEPTH - total number of words in each buffer
//    IMG_WIDTH - frame width in words (= pixels per row / 4)
// ============================================================
module pixel_acc #(
    parameter int BUF_AW    = 15,  // buffer adress width
    parameter int BUF_DEPTH = 25344,
    parameter int IMG_WIDTH = 88      // 352 pixels / 4 pixels-per-word
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // ibuf read port
    output logic                  ibuf_rd_en,
    output logic [BUF_AW-1:0]     ibuf_rd_addr,
    input  logic [31:0]           ibuf_rd_data,

    // obuf write port
    output logic                  obuf_wr_en,
    output logic [BUF_AW-1:0]     obuf_wr_addr,
    output logic [31:0]           obuf_wr_data,



    // handshake
    input  logic                  start,
    output logic                  finish
);

    // ============================================================
    // BRAM has 1-cycle read latency, so:
    //   cycle 0: read ibuf[0] issued  (rd_data not yet valid)
    //   cycle 1: read ibuf[1] issued, ibuf[0] data valid -> write obuf[0]
    //   ...
    //   cycle N: read ibuf[N] issued (last), write obuf[N-1]
    //   cycle N+1: no new read, write obuf[N], assert finish
    //
    // ============================================================

    typedef enum logic [2:0] {
        IDLE    = 3'd0,
        LOAD_R0_R1_R2 = 3'd1,
        CALC_POP = 3'd5,
        END_ROW = 3'd6,
        FINAL = 3'd7
    } state_t;

    state_t state, next_state;

    // optimisation:
    // port for the buffer middle line
    logic                  lbu_wr_en;
    logic [6:0]            lbu_wr_addr;
    logic [31:0]           lbu_wr_data;
    logic                  lbu_rd_en;
    logic [6:0]            lbu_rd_addr;
    logic [31:0]           lbu_rd_data;

    // port for the buffer middle line
    logic                  lbm_wr_en;
    logic [6:0]            lbm_wr_addr;
    logic [31:0]           lbm_wr_data;
    logic                  lbm_rd_en;
    logic [6:0]            lbm_rd_addr;
    logic [31:0]           lbm_rd_data;

    // Optimisation: save the two last row in two buffers
    bram_sdp #(.DATA_WIDTH(32), .ADDR_WIDTH(7), .DEPTH(IMG_WIDTH)) LineBufferUp ( // 2^6< 88 <2^7 -> need 7 bits for adressing
      .clk     (clk),
      .wr_en   (lbu_wr_en),
      .wr_addr (lbu_wr_addr),
      .wr_data (lbu_wr_data),
      .rd_en   (lbu_rd_en),
      .rd_addr (lbu_rd_addr),
      .rd_data (lbu_rd_data)
  );
    bram_sdp #(.DATA_WIDTH(32), .ADDR_WIDTH(7), .DEPTH(IMG_WIDTH)) LineBufferMid ( // 2^6< 88 <2^7 -> need 7 bits for adressing
      .clk     (clk),
      .wr_en   (lbm_wr_en),
      .wr_addr (lbm_wr_addr),
      .wr_data (lbm_wr_data),
      .rd_en   (lbm_rd_en),
      .rd_addr (lbm_rd_addr),
      .rd_data (lbm_rd_data)
  );

    // COUNTERS:
    logic [8:0] r, next_r; //row counter: 1 to 286
    logic [6:0] w, next_w; //word counter: 352(imgwidth)/4(addresswidth): 0 to 87

    // DATAPATH REGISTERS: 3x4 current pixel-window + 3x2 overlap + 2 output
    // holding registers.
    logic [7:0] curr [3][4];
    logic [7:0] next_curr [3][4];
    logic [7:0] prev_c [3], prev_d [3];
    logic [7:0] next_prev_c [3], next_prev_d [3];
    logic [7:0] hold_b, hold_c, hold_a;
    logic [7:0] next_hold_b, next_hold_c, next_hold_a;

    


    // SOBEL FUNCTION (COMBINATIONAL)
    function automatic [7:0] sobel_calc(
      input logic [7:0] p00, p01, p02,
      input logic [7:0] p10, p11, p12,
      input logic [7:0] p20, p21, p22
    );

      int signed gx, gy, abs_gx, abs_gy, sum;
      gx = (int'(p02) + 2*int'(p12) + int'(p22)) - (int'(p00) + 2*int'(p10) + int'(p20));
      gy = (int'(p20) + 2*int'(p21) + int'(p22)) - (int'(p00) + 2*int'(p01) + int'(p02));
      abs_gx = (gx < 0) ? -gx : gx;
      abs_gy = (gy < 0) ? -gy : gy;
      sum = abs_gx + abs_gy;
      return (sum > 255) ? 8'd255 : sum[7:0];
    endfunction

    // SEQUENTIAL LOGIC.
    always_ff @(posedge clk or negedge rst_n) begin
      if (~rst_n) begin
        state <= IDLE;
        r <= 9'd1;
        w <= 7'd0;
        hold_b <= 8'd0;
        hold_c <= 8'd0;
        hold_a <= 8'd0;
        for (int i = 0; i < 3; i++) begin
          prev_c[i] <= 8'd0;
          prev_d[i] <= 8'd0;
          for (int j = 0; j < 4; j++) begin
            curr[i][j] <= 8'd0;
          end
        end
      end else begin
        state <= next_state;
        r <= next_r;
        w <= next_w;
        hold_b <= next_hold_b;
        hold_c <= next_hold_c;
        hold_a <= next_hold_a;
        for (int i = 0; i < 3; i++) begin
          prev_c[i] <= next_prev_c[i];
          prev_d[i] <= next_prev_d[i];
          for (int j = 0; j < 4; j++) begin
            curr[i][j] <= next_curr[i][j];
          end
        end
      end
    end

    // COMBINATIONAL LOGIC (ASMD).
    always_comb begin
      next_state = state;
      next_r = r;
      next_w = w;
      next_hold_b = hold_b;
      next_hold_c = hold_c;
      next_hold_a = hold_a;
      for (int i = 0; i < 3; i++) begin
        next_prev_c[i] = prev_c[i];
        next_prev_d[i] = prev_d[i];
        for (int j = 0; j < 4; j++) begin
          next_curr[i][j] = curr[i][j];
        end
      end

      ibuf_rd_en = 1'b0;
      ibuf_rd_addr = '0;
      obuf_wr_en = 1'b0;
      obuf_wr_addr = '0;
      obuf_wr_data = '0;
      finish = 1'b0;

      // optimisation:
      // buffer up
      lbu_wr_en = 1'b0;
      lbu_wr_addr = '0;
      lbu_wr_data = '0;
      lbu_rd_en = 1'b0;
      lbu_rd_addr = '0;

      //buffer middle
      lbm_wr_en = 1'b0;
      lbm_wr_addr = '0;
      lbm_wr_data = '0;
      lbm_rd_en = 1'b0;
      lbm_rd_addr = '0;

      // Sobel outputs computed combinationally from active 3x6 window:
      // window d_prev: [prev_c, prev_d, curr_a]
      // window a_curr: [prev_d, curr_a, curr_b]
      // window b_curr: [curr_a, curr_b, curr_c]
      // window c_curr: [curr_b, curr_c, curr_d]
      //
      // Row i has cols: prev_c[i], prev_d[i], curr_a[i], curr_b[i], curr_c[i],
      // curr_d[i].
      
      case (state)
        IDLE: begin
          if (start == 1) begin
            next_r = 9'd1;
            next_w = 7'd0;
            ibuf_rd_en = 1'b1;
            ibuf_rd_addr = (r - 1) * IMG_WIDTH + w; // request R0.
            next_state = LOAD_R0_R1_R2;
          end
        end

        LOAD_R0_R1_R2: begin
          next_curr[0][0] = lbu_rd_data[7:0]; // load R0.
          next_curr[0][1] = lbu_rd_data[15:8];
          next_curr[0][2] = lbu_rd_data[23:16];
          next_curr[0][3] = lbu_rd_data[31:24];

          

          next_curr[1][0] = lbm_rd_data[7:0]; // load R1.
          next_curr[1][1] = lbm_rd_data[15:8];
          next_curr[1][2] = lbm_rd_data[23:16];
          next_curr[1][3] = lbm_rd_data[31:24];

        
          next_curr[2][0] = ibuf_rd_data[7:0]; // load R2.
          next_curr[2][1] = ibuf_rd_data[15:8];
          next_curr[2][2] = ibuf_rd_data[23:16];
          next_curr[2][3] = ibuf_rd_data[31:24];

          
          

          next_state = CALC_POP;

          

        end

        CALC_POP: begin
          


          // calculate 4 sobel outputs.
          // d_prev: col 1, 2, 3 -> prev_c[i], prev_d[i], curr[i][0]
          // a_curr: col 2, 3, 4 -> prev_d[i], curr[i][0], curr[i][1]
          // b_curr: col 3, 4, 5 -> curr[i][0], curr[i][1], curr[i][2]
          // c_curr: col 4, 5, 6 -> curr[i][1], curr[i][2], curr[i][3]

          automatic logic [7:0] sobel_d_prev = sobel_calc(
            prev_c[0], prev_d[0], curr[0][0],
            prev_c[1], prev_d[1], curr[1][0],
            prev_c[2], prev_d[2], curr[2][0]
          );

          automatic logic [7:0] sobel_a_curr = sobel_calc(
            prev_d[0], curr[0][0], curr[0][1],
            prev_d[1], curr[1][0], curr[1][1],
            prev_d[2], curr[2][0], curr[2][1]
          );

          automatic logic [7:0] sobel_b_curr = sobel_calc(
            curr[0][0], curr[0][1], curr[0][2],
            curr[1][0], curr[1][1], curr[1][2],
            curr[2][0], curr[2][1], curr[2][2]
          );

          automatic logic [7:0] sobel_c_curr = sobel_calc(
            curr[0][1], curr[0][2], curr[0][3],
            curr[1][1], curr[1][2], curr[1][3],
            curr[2][1], curr[2][2], curr[2][3]
          );

          // optimisation: 
          // store the middle row (a, b, c and d ) in the buffer for the up line, these data will be use for the next row (same column)
          lbu_wr_en = 1'b1;
          lbu_wr_addr = w;
          lbu_wr_data = lbm_rd_data;

          // store the lower row (a, b, c and d ) in the buffer for the middle line,
          lbm_wr_en = 1'b1;
          lbm_wr_addr = w;
          lbm_wr_data = ibuf_rd_data;

          // save b and c for next time.
          next_hold_b = sobel_b_curr;
          next_hold_c = sobel_c_curr;
          next_hold_a = sobel_a_curr;

          // shift current c and d into prev_c and prev_d.
          for (int i = 0; i < 3; i++) begin
            next_prev_c[i] = curr[i][2]; // keep prev c for the next window so prev d can be calculated
            next_prev_d[i] = curr[i][3]; // keep prev d so that it can be calculated in the next window.
          end

          // handle output writing to obuf (skipped on w = 0 since only
          // 2 pixels ready).
          if (w > 0) begin // if we are in the next window, that means prev d is already done and hold a,b,c can be used to send 4 sobel pixels over obuf.
            obuf_wr_en = 1'b1;
            obuf_wr_addr = r * IMG_WIDTH + (w - 1); // load next window.
            obuf_wr_data = {sobel_d_prev, hold_c, hold_b, hold_a}; // while loading next window, send previous 4 calculated sobel pixels.
          end

          if (w == IMG_WIDTH - 1) begin // if row ends, move to next row.
            next_state = END_ROW;
          end else begin // not the last word of the row? then load next word in same row until row ends.
            next_w = w + 1'b1;
            lbu_rd_en = 1'b1;
            lbu_rd_addr = w + 1;
            lbm_rd_en = 1'b1;
            lbm_rd_addr = w + 1;
            ibuf_rd_en = 1'b1;
            ibuf_rd_addr = (r + 1) * IMG_WIDTH + (w+1);
            next_state = LOAD_R0_R1_R2;
          end
        end

        END_ROW: begin
          // flush final 2 leftover pixels of the row.
          obuf_wr_en = 1'b1;
          obuf_wr_addr = r * IMG_WIDTH + (IMG_WIDTH - 1);
          obuf_wr_data = {8'd0, hold_c, hold_b, hold_a}; // last d col in the last word of the previous row is not considered.
          next_w = 7'd0;

          // clear overlap regs for the start of the next row.
          /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //BORDER DOES NOT CALCULATE LAST D OF LAST WORD IN PREVIOUS ROW.
          for (int i = 0; i < 3; i++) begin
            next_prev_c[i] = 8'd0; // prev_c is not saved because we will not calculate prev_d from the last word of the previous row.
            next_prev_d[i] = 8'd0; // we dont consider prev_d of the last word in the previous row.
          end

          if (r == 9'd286) begin // check if we are at the last word in the last row.
            next_state = FINAL; // if so then signal DONE.
          end else begin // if not last row yet just continue to the next row.
            next_r = r + 1'b1;
            lbu_rd_en = 1'b1;
            lbu_rd_addr = 0;
            lbm_rd_en = 1'b1;
            lbm_rd_addr = 0;
            ibuf_rd_en = 1'b1;
            ibuf_rd_addr =  (r + 2) * IMG_WIDTH; // start of next row minus 1.
            next_state = LOAD_R0_R1_R2;
          end
        end

        FINAL: begin
          finish = 1'b1;
          next_state = IDLE;
        end

        default: next_state = IDLE;
      endcase
    end

endmodule
