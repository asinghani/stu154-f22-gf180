`define default_netname none

// Student TinyTapeout Designs

module tt_ngaertne (
    input  logic [7:0] io_in,
    output logic [7:0] io_out
);
  logic clock, reset;
  assign clock = io_in[0];  //clock
  assign reset = io_in[1];  //reset to clear everything
  typedef enum logic [3:0] {
    LOAD = 4'd0, //loads a value from data file into operation register
    STORE = 4'd1, //stores value from operation register to data file
    ADD = 4'd2, //adds datac to operation register value
    MUL = 4'd3, //multiples operation register value by datac
    SUB = 4'd4, //subtracts datac from operation register value
    SHIFTL = 4'd5, //shifts operation register value left by datac or 3, 
    //whichever is less
    SHIFTR = 4'd6, //shifts operation register value right by datac or 3,
    //whichever is less
    JUMPTOIF = 4'd7, //jumps pc to data[value] if io_in[7] is a 1, else 
    //does nothing
    LOGICAND = 4'd8,
    //logical and between operation register value and datac
    LOGICOR = 4'd9,
    //logical or between operation register value and datac
    EQUALS = 4'd10,
    //equality check between operation register value and datac
    NEQ = 4'd11,
    //inequality check between operation register value and datac
    BITAND = 4'd12,
    //bitwise and between operation register value and datac    
    BITOR = 4'd13,
    //bitwise or between operation register value and datac
    LOGICNOT = 4'd14,
    //logical not on operation register value 
    BITNOT = 4'd15
    //bitwise not on operation register value
  } prog_t;
  //yosys doesn't like it if i enum directly instead of typedef w/ memories for 
  //some reason
  prog_t prog[15:0];  //program storage "file"
  logic [3:0] data[15:0];  //data storage :file
  enum logic [1:0] {
    LOADPROG = 2'd0, //loads a program into the program "file"
    LOADDATA = 2'd1, //loads data into the data "file"
    SETRUNPT = 2'd2, //designed to be used right before run, but can also be used to input additional data i guess
    RUNPROG  = 2'd3 //run the program
  } instruction;
  assign instruction = io_in[3:2];  //current instruction
  logic [3:0] pc;  //program counter
  logic [3:0] datac;
  prog_t progc;
  assign progc = prog[pc]; //prog at pc
  assign datac = data[pc]; //data at pc
  logic [3:0] regval;  //current value being operated on (accumulator)
  assign io_out = {regval, pc};
  logic [3:0] inputdata;
  logic lastdata3;  //input data
  assign inputdata = io_in[7:4];
  logic [3:0] npc;
  assign npc = pc+1;

  always_ff @(posedge clock) begin
    if (!reset) begin
      lastdata3 <= inputdata[3];
      case (instruction)

        LOADPROG: begin  //loads a program into the program "file"
          prog[pc] <= inputdata;
          pc <= npc;
        end
        LOADDATA: begin  //loads data into the data "file"
          data[pc] <= inputdata;
          pc <= npc;
        end
        SETRUNPT: begin //designed to be used right before run, but can also be used to input additional data i guess
          pc <= inputdata;
        end
        RUNPROG: begin  //run the program
          case (progc)
            LOAD: begin
              //loads a value from the data file
              regval <= datac;
              pc <= npc;
            end
            STORE: begin
              //stores a value into the data file
              data[datac] <= regval;
              pc <= npc;
            end
            ADD: begin
              //adds the value at the appropriate data address to the register
              regval <= regval + datac;
              pc <= npc;
            end
            SUB: begin
              //subtracts the value at the appropriate addr from the register
              regval <= regval - datac;
              pc <= npc;
            end
            MUL: begin
              //multiplies the register by the value at the appropriate addr
              pc <= npc;
              regval <= regval * datac;
            end
            SHIFTL: begin
              //shifts the register left by the value at the appropriate addr, 
              //or 3, whichever is less.
              pc <= npc;
              regval <= ((datac<4) ? regval<<datac : regval << 3);
            end
            SHIFTR: begin
              //shifts the register right by the value at the appropriate addr, 
              //or 3, whichever is less
              pc <= npc;
              regval <= ((datac<4) ? regval>>datac : regval >> 3);
            end
            JUMPTOIF: //jumps to value if input pin 7 is a one
              //not unconditional to avoid looping forever
              //weird external condition because of single-register/stack 
              //design due to space limits & effective max of 16 
              //microinstructions, which is theoretically circumventable
              //by serializing IO and reassembling, but that takes too much 
              //space
            begin
              pc <= (lastdata3) ? datac : npc;
              regval <= regval;
            end
            LOGICAND: begin
              //logical and between regval and datac
              pc <= npc;
              regval <= regval && datac;
            end
            LOGICOR: begin
              //logical or between regval and datac
              pc <= npc;
              regval <= regval || datac;
            end
            EQUALS: begin
              //equality check between regval and datac
              pc <= npc;
              regval <= (regval == datac);
            end
            NEQ: begin
              //inequality check between regval and datac
              pc <= npc;
              regval <= (regval != datac);
            end
            BITAND: begin
              //bitwise and between regval and datac
              pc <= npc;
              regval <= (regval & datac);
            end
            BITNOT: begin
              //bitwise inversion on regval
              pc<= npc;
              regval <= ~(regval);
            end
            BITOR: begin
              //bitwise or between regval and datac
              pc<= npc;
              regval <= (regval | datac);
            end
            LOGICNOT: begin
              //logical NOT on regval
              pc <= npc;
              regval <= !regval;
            end
          endcase
        end
      endcase
    end else begin
      pc <= 0;
      regval <= 0;
      data[0] <= 4'd0;
      data[1] <= 4'd0;
      data[2] <= 4'd0;
      data[3] <= 4'd0;
      data[4] <= 4'd0;
      data[5] <= 4'd0;
      data[6] <= 4'd0;
      data[7] <= 4'd0;
      data[8] <= 4'd0;
      data[9] <= 4'd0;
      data[10] <= 4'd0;
      data[11] <= 4'd0;
      data[12] <= 4'd0;
      data[13] <= 4'd0;
      data[14] <= 4'd0;
      data[15] <= 4'd0;
      prog[0] <= 4'd0;
      prog[1] <= 4'd0;
      prog[2] <= 4'd0;
      prog[3] <= 4'd0;
      prog[4] <= 4'd0;
      prog[5] <= 4'd0;
      prog[6] <= 4'd0;
      prog[7] <= 4'd0;
      prog[8] <= 4'd0;
      prog[9] <= 4'd0;
      prog[10] <= 4'd0;
      prog[11] <= 4'd0;
      prog[12] <= 4'd0;
      prog[13] <= 4'd0;
      prog[14] <= 4'd0;
      prog[15] <= 4'd0;
      lastdata3 <= 1'd0;
    end
  end
endmodule



module tt_jrecta (
	input [7:0] io_in,
	output [7:0] io_out
);


  async_fifo #(.WIDTH(3), .DEPTH(8)) top
    (.rst(io_in[2]),

     .wclk(io_in[0]), .we(io_in[3]),
     .full(io_out[3]),
     .wdata(io_in[7:5]),

     .rclk(io_in[1]), .re(io_in[4]),
     .empty(io_out[4]),
     .rdata(io_out[7:5]));
endmodule

module async_fifo
  #(parameter WIDTH=4,
    parameter DEPTH=4)
  (input logic rst,
   input logic wclk, we,
   output logic full,
   input logic[WIDTH-1:0] wdata,
   input logic rclk, re,
   output logic empty,
   output logic[WIDTH-1:0] rdata);

  // extra bit for full detection
  parameter PTR_WIDTH = $clog2(DEPTH)+1;

  logic [PTR_WIDTH-1:0] wptr, wptr_gray, rptr, rptr_gray;
  logic [WIDTH-1:0] data[DEPTH-1:0];

  // store data
  always_ff @(posedge wclk)
    if(we & ~full)
      data[wptr[PTR_WIDTH-2:0]] <= wdata;
  assign rdata = data[rptr[PTR_WIDTH-2:0]];

  // all logic in write domain
  write_half #(PTR_WIDTH) frontend
    (.rst, .wclk, .we,
     .rptr_gray,
     .wptr, .wptr_gray,
     .full);

  // all logic in read domain
  read_half #(PTR_WIDTH) backend
    (.rst, .rclk, .re,
     .wptr_gray,
     .rptr, .rptr_gray,
     .empty);
endmodule: async_fifo

module read_half
  #(parameter PTR_WIDTH)
  (input logic rst, rclk, re,
   input logic [PTR_WIDTH-1:0] wptr_gray,
   output logic [PTR_WIDTH-1:0] rptr, rptr_gray,
   output logic empty);

  logic [PTR_WIDTH-1:0] wptr_gray1, wptr_gray2;

  // rptr counter
  reg_ar #(PTR_WIDTH) rptr_reg
    (.rst, .clk(rclk), .en(re & ~empty),
     .d(rptr + (PTR_WIDTH)'(1)), .q(rptr));

  // sync wptr_gray
  reg_ar #(2*PTR_WIDTH) wptr_gray_sync
    (.rst, .clk(rclk), .en('1),
     .d({wptr_gray, wptr_gray1}), .q({wptr_gray1, wptr_gray2}));

  // generate gray code
  bin2gray #(PTR_WIDTH) rptr_b2g
    (.binary(rptr), .gray(rptr_gray));

  // empty is easy to check
  assign empty = wptr_gray2 == rptr_gray;
endmodule

module write_half
  #(parameter PTR_WIDTH)
  (input logic rst, wclk, we,
   input logic [PTR_WIDTH-1:0] rptr_gray,
   output logic [PTR_WIDTH-1:0] wptr, wptr_gray,
   output logic full);

  logic [PTR_WIDTH-1:0] rptr_gray1, rptr_gray2;

  // wptr counter
  reg_ar #(PTR_WIDTH) wptr_reg
    (.rst, .clk(wclk), .en(we & ~full),
     .d(wptr + (PTR_WIDTH)'(1)), .q(wptr));

  // sync rptr_gray
  reg_ar #(2*PTR_WIDTH) rptr_gray_sync
    (.rst, .clk(wclk), .en('1),
     .d({rptr_gray, rptr_gray1}), .q({rptr_gray1, rptr_gray2}));

  // generate gray code
  bin2gray #(PTR_WIDTH) wptr_b2g
    (.binary(wptr), .gray(wptr_gray));

  // grey code math...
  if(PTR_WIDTH > 2)
    assign full = rptr_gray2[PTR_WIDTH-1 -: 2] == ~wptr_gray[PTR_WIDTH-1 -: 2]
                  && rptr_gray2[0 +: (PTR_WIDTH-2)] == wptr_gray[0 +: (PTR_WIDTH-2)];
  else
    assign full = rptr_gray2 == ~wptr_gray;
endmodule

module reg_ar
  #(parameter WIDTH)
  (input logic clk, rst, en,
   input logic[WIDTH-1:0] d,
   output logic[WIDTH-1:0] q);
  always_ff @(posedge clk, posedge rst)
    if(rst)
      q <= '0;
    else if(en)
      q <= d;
endmodule // reg_ar

module gray2bin
  #(parameter WIDTH)
  (input logic[WIDTH-1:0] gray,
   output logic[WIDTH-1:0] binary);

  generate for(genvar i = 0; i < WIDTH-1; i++)
    assign binary[i] = gray[i] ^ binary[i+1];
  endgenerate
  assign binary[WIDTH-1] = gray[WIDTH-1];
endmodule // gray2bin

module bin2gray
  #(parameter WIDTH)
  (input logic[WIDTH-1:0] binary,
   output logic [WIDTH-1:0] gray);

  assign gray = binary ^ (binary >> 1);
endmodule // bin2gray

module tt_sophiali (
  input logic [7:0] io_in,
  output logic [7:0] io_out  
);
  
  logic clock, reset, en;
  logic [2:0] in;
  logic [1:0] arithOp;
  
  always_comb begin
    clock = io_in[0];
    reset = io_in[1];
    en = io_in[2];
    in = io_in[3:5];
    arithOp = io_in[6:7];
  end

  logic enable;
  enum logic {IDLE, GO} state, nextState;

  // ALU
  always_ff @(posedge clock, posedge reset) begin
    if (reset)
      io_out <= 0;
    else begin
      if (enable)
        unique case (arithOp)
          2'b00: io_out <= io_out + in;  // add
          2'b01: io_out <= io_out - in;  // subtract
          2'b10: io_out <= io_out ^ {5'b0, in};  // XOR per bit
          2'b11: io_out <= io_out << in;  // Left-shift per bit
        endcase
    end
  end

  // FF to ensure button for enable isn't continuously set
  always_ff @(posedge clock, posedge reset) begin
    if (reset)
      state <= IDLE;
    else begin
      state <= nextState; 
    end
  end

  assign enable = (state == IDLE && nextState == GO);
  always_comb begin
    if (state == IDLE) begin
      nextState = (en) ? GO : IDLE;
    end
    else
      nextState = (en) ? GO : IDLE;
  end

endmodule

module tt_mgee3 (
  input [7:0] io_in,
  output [7:0] io_out
);
  wire net1 = io_in[0];
  wire net2 = io_in[1];
  wire net3 = io_in[2];
  wire net4 = io_in[3];
  wire net5 = io_in[4];
  wire net6 = io_in[5];
  wire net7 = 1'b1;
  wire net8;
  wire net9 = 1'b0;
  wire net10;
  wire net11;
  wire net12;
  wire net13;
  wire net14;
  wire net15;
  wire net16;
  wire net17;
  wire net18;
  wire net19;
  wire net20;
  wire net21 = 1'b0;

  xor_cell gate11 (
    .a (net2),
    .b (net5),
    .out (net8)
  );
  xor_cell gate7 (
    .a (net8),
    .b (net10),
    .out (net11)
  );
  and_cell gate8 (
    .a (net8),
    .b (net10),
    .out (net12)
  );
  and_cell gate9 (
    .a (net2),
    .b (net5),
    .out (net13)
  );
  or_cell gate10 (
    .a (net12),
    .b (net13),
    .out (net14)
  );
  xor_cell gate1 (
    .a (net3),
    .b (net6),
    .out (net15)
  );
  xor_cell gate2 (
    .a (net15),
    .b (net14),
    .out (net16)
  );
  and_cell gate3 (
    .a (net15),
    .b (net14),
    .out (net17)
  );
  and_cell gate4 (
    .a (net3),
    .b (net6),
    .out (net18)
  );
  or_cell gate5 (
    .a (net17),
    .b (net18),
    .out (net19)
  );
  and_cell gate6 (
    .a (net1),
    .b (net4),
    .out (net10)
  );
  xor_cell gate12 (
    .a (net1),
    .b (net4),
    .out (net20)
  );
endmodule


module tt_jxlu (
    input logic [7:0] io_in,
    output logic [7:0] io_out
);
    logic clk;
    logic reset;
    logic [5:0] duty;
    logic pwm_signal;
	
	assign io_out = pwm_signal;
	assign clk = io_in[0];
	assign reset = io_in[1];
	assign duty = io_in[7:2];


    logic [5:0] counter;
    logic [5:0] active_duty;

    always_ff @(posedge clk) begin
        if (reset == 1) begin
            if (duty > 6'd50) begin
                active_duty <= 6'd50;
            end
            else begin
                active_duty <= duty;
            end

            counter <= 0;
        end
        else begin
            counter <= counter + 1'b1;

            if (counter == 49) begin
                counter <= 0;
            end

            if (counter >= active_duty) begin
                pwm_signal <= 1'b0;
            end
            else begin
                pwm_signal <= 1'b1;
            end
        end
    end

endmodule


`default_nettype none

module tt_qilins (
  input [7:0] io_in,
  output [7:0] io_out
);
  wire net1 = io_in[0];
  wire net2 = io_in[1];
  wire net3 = io_in[2];
  wire net4 = io_in[3];
  wire net5;
  wire net6;
  wire net7;
  wire net8;
  wire net9;
  wire net10;
  wire net11;
  wire net12;
  wire net13 = 1'b0;
  wire net14 = 1'b1;
  wire net15;
  wire net16;
  wire net17;
  wire net18;
  wire net19;
  wire net20;
  wire net21;
  wire net22;
  wire net23;
  wire net24;
  wire net25;
  wire net26;
  wire net27;
  wire net28;
  wire net29;
  wire net30;
  wire net31;
  wire net32;
  wire net33;
  wire net34;
  wire net35;
  wire net36;
  wire net37;
  wire net38;
  wire net39;
  wire net40;
  wire net41;
  wire net42;
  wire net43;
  wire net44;
  wire net45;
  wire net46;
  wire net47;
  wire net48;
  wire net49;
  wire net50;
  wire net51;
  wire net52;
  wire net53;
  wire net54;
  wire net55;
  wire net56;
  wire net57;
  wire net58;
  wire net59;
  wire net60;
  wire net61;
  wire net62;
  wire net63;
  wire net64;
  wire net65;
  wire net66;
  wire net67;
  wire net68;
  wire net69;
  wire net70;
  wire net71;
  wire net72;
  wire net73;
  wire net74;
  wire net75;
  wire net76;
  wire net77;
  wire net78;
  wire net79;
  wire net80;
  wire net81;
  wire net82;
  wire net83;
  wire net84;
  wire net85;
  wire net86;

  assign io_out[0] = net5;
  assign io_out[1] = net6;
  assign io_out[2] = net7;
  assign io_out[3] = net8;
  assign io_out[4] = net9;
  assign io_out[5] = net10;
  assign io_out[6] = net11;
  assign io_out[7] = net12;

  not_cell gate7 (
    .in (net1),
    .out (net15)
  );
  not_cell gate8 (
    .in (net2),
    .out (net16)
  );
  not_cell gate9 (
    .in (net3),
    .out (net17)
  );
  not_cell gate10 (
    .in (net4),
    .out (net18)
  );
  and_cell gate11 (
    .a (net15),
    .b (net2),
    .out (net19)
  );
  and_cell gate14 (
    .a (net17),
    .b (net19),
    .out (net20)
  );
  and_cell gate12 (
    .a (net17),
    .b (net18),
    .out (net21)
  );
  and_cell gate15 (
    .a (net2),
    .b (net18),
    .out (net22)
  );
  and_cell gate16 (
    .a (net1),
    .b (net16),
    .out (net23)
  );
  and_cell gate17 (
    .a (net1),
    .b (net3),
    .out (net24)
  );
  or_cell gate18 (
    .a (net20),
    .b (net21),
    .out (net25)
  );
  or_cell gate19 (
    .a (net22),
    .b (net23),
    .out (net26)
  );
  or_cell gate20 (
    .a (net25),
    .b (net27),
    .out (net10)
  );
  or_cell gate21 (
    .a (net26),
    .b (net24),
    .out (net27)
  );
  and_cell gate22 (
    .a (net1),
    .b (net16),
    .out (net28)
  );
  and_cell gate23 (
    .a (net17),
    .b (net28),
    .out (net29)
  );
  and_cell gate24 (
    .a (net15),
    .b (net2),
    .out (net30)
  );
  and_cell gate25 (
    .a (net30),
    .b (net4),
    .out (net31)
  );
  and_cell gate26 (
    .a (net1),
    .b (net18),
    .out (net32)
  );
  and_cell gate27 (
    .a (net15),
    .b (net3),
    .out (net33)
  );
  and_cell gate28 (
    .a (net2),
    .b (net3),
    .out (net34)
  );
  and_cell gate29 (
    .a (net16),
    .b (net18),
    .out (net35)
  );
  or_cell gate30 (
    .a (net29),
    .b (net31),
    .out (net36)
  );
  or_cell gate31 (
    .a (net32),
    .b (net33),
    .out (net37)
  );
  or_cell gate32 (
    .a (net34),
    .b (net35),
    .out (net38)
  );
  or_cell gate33 (
    .a (net36),
    .b (net37),
    .out (net39)
  );
  or_cell gate34 (
    .a (net39),
    .b (net38),
    .out (net5)
  );
  and_cell gate13 (
    .a (net15),
    .b (net17),
    .out (net40)
  );
  and_cell gate35 (
    .a (net18),
    .b (net40),
    .out (net41)
  );
  and_cell gate36 (
    .a (net15),
    .b (net3),
    .out (net42)
  );
  and_cell gate37 (
    .a (net4),
    .b (net42),
    .out (net43)
  );
  and_cell gate38 (
    .a (net1),
    .b (net17),
    .out (net44)
  );
  and_cell gate39 (
    .a (net4),
    .b (net44),
    .out (net45)
  );
  and_cell gate40 (
    .a (net16),
    .b (net17),
    .out (net46)
  );
  and_cell gate41 (
    .a (net16),
    .b (net18),
    .out (net47)
  );
  or_cell gate42 (
    .a (net41),
    .b (net43),
    .out (net48)
  );
  or_cell gate43 (
    .a (net45),
    .b (net46),
    .out (net49)
  );
  or_cell gate44 (
    .a (net48),
    .b (net49),
    .out (net50)
  );
  or_cell gate45 (
    .a (net50),
    .b (net47),
    .out (net6)
  );
  and_cell gate46 (
    .a (net15),
    .b (net17),
    .out (net51)
  );
  and_cell gate47 (
    .a (net15),
    .b (net4),
    .out (net52)
  );
  and_cell gate48 (
    .a (net17),
    .b (net4),
    .out (net53)
  );
  and_cell gate49 (
    .a (net15),
    .b (net2),
    .out (net54)
  );
  and_cell gate50 (
    .a (net1),
    .b (net16),
    .out (net55)
  );
  or_cell gate51 (
    .a (net51),
    .b (net52),
    .out (net56)
  );
  or_cell gate52 (
    .a (net53),
    .b (net54),
    .out (net57)
  );
  or_cell gate53 (
    .a (net58),
    .b (net55),
    .out (net7)
  );
  or_cell gate54 (
    .a (net56),
    .b (net57),
    .out (net58)
  );
  and_cell gate55 (
    .a (net15),
    .b (net16),
    .out (net59)
  );
  and_cell gate56 (
    .a (net59),
    .b (net18),
    .out (net60)
  );
  and_cell gate57 (
    .a (net16),
    .b (net3),
    .out (net61)
  );
  and_cell gate58 (
    .a (net61),
    .b (net4),
    .out (net62)
  );
  and_cell gate59 (
    .a (net2),
    .b (net17),
    .out (net63)
  );
  and_cell gate60 (
    .a (net63),
    .b (net4),
    .out (net64)
  );
  and_cell gate61 (
    .a (net2),
    .b (net3),
    .out (net65)
  );
  and_cell gate62 (
    .a (net65),
    .b (net18),
    .out (net66)
  );
  and_cell gate63 (
    .a (net1),
    .b (net17),
    .out (net67)
  );
  or_cell gate64 (
    .a (net60),
    .b (net62),
    .out (net68)
  );
  or_cell gate65 (
    .a (net68),
    .b (net69),
    .out (net70)
  );
  or_cell gate66 (
    .a (net64),
    .b (net66),
    .out (net69)
  );
  or_cell gate68 (
    .a (net70),
    .b (net67),
    .out (net8)
  );
  and_cell gate67 (
    .a (net16),
    .b (net18),
    .out (net71)
  );
  and_cell gate69 (
    .a (net3),
    .b (net18),
    .out (net72)
  );
  and_cell gate70 (
    .a (net1),
    .b (net3),
    .out (net73)
  );
  and_cell gate71 (
    .a (net1),
    .b (net2),
    .out (net74)
  );
  or_cell gate72 (
    .a (net71),
    .b (net72),
    .out (net75)
  );
  or_cell gate73 (
    .a (net73),
    .b (net74),
    .out (net76)
  );
  or_cell gate74 (
    .a (net75),
    .b (net76),
    .out (net9)
  );
  and_cell gate75 (
    .a (net15),
    .b (net2),
    .out (net77)
  );
  and_cell gate76 (
    .a (net77),
    .b (net17),
    .out (net78)
  );
  and_cell gate77 (
    .a (net16),
    .b (net3),
    .out (net79)
  );
  and_cell gate78 (
    .a (net3),
    .b (net18),
    .out (net80)
  );
  and_cell gate79 (
    .a (net1),
    .b (net16),
    .out (net81)
  );
  and_cell gate80 (
    .a (net1),
    .b (net4),
    .out (net82)
  );
  or_cell gate81 (
    .a (net78),
    .b (net79),
    .out (net83)
  );
  or_cell gate82 (
    .a (net80),
    .b (net81),
    .out (net84)
  );
  or_cell gate83 (
    .a (net85),
    .b (net82),
    .out (net11)
  );
  or_cell gate84 (
    .a (net83),
    .b (net84),
    .out (net85)
  );
  and_cell gate85 (
    .a (net1),
    .b (net86),
    .out (net12)
  );
  nand_cell gate87 (
    .a (net17),
    .b (net16),
    .out (net86)
  );
endmodule


module buffer_cell (
    input wire in,
    output wire out
    );
    assign out = in;
endmodule

module and_cell (
    input wire a,
    input wire b,
    output wire out
    );

    assign out = a & b;
endmodule

module or_cell (
    input wire a,
    input wire b,
    output wire out
    );

    assign out = a | b;
endmodule

module xor_cell (
    input wire a,
    input wire b,
    output wire out
    );

    assign out = a ^ b;
endmodule

module nand_cell (
    input wire a,
    input wire b,
    output wire out
    );

    assign out = !(a&b);
endmodule

module not_cell (
    input wire in,
    output wire out
    );

    assign out = !in;
endmodule

module mux_cell (
    input wire a,
    input wire b,
    input wire sel,
    output wire out
    );

    assign out = sel ? b : a;
endmodule

module dff_cell (
    input wire clk,
    input wire d,
    output reg q,
    output wire notq
    );

    assign notq = !q;
    always @(posedge clk)
        q <= d;

endmodule

module dffsr_cell (
    input wire clk,
    input wire d,
    input wire s,
    input wire r,
    output reg q,
    output wire notq
    );

    assign notq = !q;

    always @(posedge clk or posedge s or posedge r) begin
        if (r)
            q <= 0;
        else if (s)
            q <= 1;
        else
            q <= d;
    end
endmodule
