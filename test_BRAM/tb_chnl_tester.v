`timescale 1 ns/10 ps
`include "chnl_tester.v"
`define CYCLE 10
`define END_CYCLE 100000
`define seed 20;

// define different test case
`define PATIN       "./in.dat" 
`define PATOUT      "./out.dat"


module tb;
  parameter NUM_OF_PAT = 1024;
  parameter MAX_LENGTH = 1024;
  parameter AMINO_ACID_WIDTH=5;
  parameter LENGTH_WIDTH = 12;
  parameter NUM_OF_SET=4;
  parameter MAX_SEGMENT = 250;
  parameter NUM_OF_SEGMENT_WIDTH = 8;
  parameter SIMILARITY_SCORE_WIDTH = 16;
  parameter WINDOW_SIZE_WIDTH = 5;
  parameter THRESHOLD_WIDTH = 6;
  parameter MAX_SEQ_NUM = 20;

  parameter WINDOW_SIZE = 17;
  parameter THRESHOLD = 18;

  integer cur_output_idx;
  reg	[15:0] golden;
  integer error;

  integer i,j;
  reg             clk;
	reg             rst;

	reg 			CHNL_RX;
	wire 			CHNL_RX_ACK;
	reg [31:0] 		CHNL_RX_LEN;
	reg [30:0] 		CHNL_RX_OFF;
	reg [127:0]		CHNL_RX_DATA;
	reg 			CHNL_RX_DATA_VALID;
	wire 			CHNL_RX_DATA_REN;

	wire 			CHNL_TX;
	reg 			CHNL_TX_ACK;
	wire 			CHNL_TX_LAST;
	wire [31:0] 	CHNL_TX_LEN;
	wire [30:0] 	CHNL_TX_OFF;
	wire [127:0] 	CHNL_TX_DATA;
	wire 			CHNL_TX_DATA_VALID;
	reg 			CHNL_TX_DATA_REN;
	reg  [31:0]		in_index;
 	integer idx_test;
  // MSA_0 accelerator in/out
  reg  [AMINO_ACID_WIDTH-1:0]               seq1, seq2;
  reg                                       length_valid_in;
  reg                                       seq1_valid_in, seq2_valid_in;
  reg  [LENGTH_WIDTH-1:0]                   seq1_length, seq2_length;
  reg                                       window_size_valid_in;
  reg  [WINDOW_SIZE_WIDTH-1:0]              window_size_in;
  reg                                       threshold_valid_in;
  reg  [THRESHOLD_WIDTH-1:0]                threshold_in;
  wire                                      valid_out;
  wire signed [SIMILARITY_SCORE_WIDTH-1:0]  similarity_score;
  wire                                      ready;


  chnl_tester #(
	 .C_PCI_DATA_WIDTH(128)
	) dut(
        .CLK(clk),
        .RST(rst),
        .CHNL_RX_CLK(),
        .CHNL_RX(CHNL_RX),
        .CHNL_RX_ACK(CHNL_RX_ACK),
        .CHNL_RX_LAST(),
        .CHNL_RX_LEN(CHNL_RX_LEN),
        .CHNL_RX_OFF(CHNL_RX_OFF),
        .CHNL_RX_DATA(CHNL_RX_DATA),
        .CHNL_RX_DATA_VALID(CHNL_RX_DATA_VALID),
        .CHNL_RX_DATA_REN(CHNL_RX_DATA_REN),
        .CHNL_TX_CLK(),
        .CHNL_TX(CHNL_TX),
        .CHNL_TX_ACK(CHNL_TX_ACK),
        .CHNL_TX_LAST(),
        .CHNL_TX_LEN(CHNL_TX_LEN),
        .CHNL_TX_OFF(CHNL_TX_OFF),
        .CHNL_TX_DATA(CHNL_TX_DATA),
        .CHNL_TX_DATA_VALID(CHNL_TX_DATA_VALID),
        .CHNL_TX_DATA_REN(CHNL_TX_DATA_REN)
    );



  // clk & rst 
  initial clk = 1'b0;
  always begin #(`CYCLE*0.5) clk = ~clk; end

	initial begin
		rst = 1'b0;
		#(`CYCLE)
		@(posedge clk); 
		#(1.0) rst = 1'b1;
		#(2*`CYCLE) rst = 1'b0;
	end

  // waveform dump
  // read pattern & golden
  reg  [127:0]  	golden_in    [NUM_OF_PAT-1:0];
	reg  [127:0]  	golden_out   [NUM_OF_PAT-1:0];
  initial begin
    // waveform dump
    `ifdef SDF
      $fsdbDumpfile("tb_syn.fsdb");
      $fsdbDumpvars(0,tb,"+mda");
      $fsdbDumpvars;
    `endif 
    `ifndef SDF
      $fsdbDumpfile("chnl_tester.fsdb");
      $fsdbDumpvars(0,tb,"+mda");
      $fsdbDumpvars;
    `endif
	$display("--------------------------- [ Simulation Starts !! ] ---------------------------");	
	$readmemh(`PATIN,  golden_in);
	$readmemh(`PATOUT, golden_out);
    

  end

// data input
integer iters,cur_input_idx,k;
initial begin
	// start RX
	CHNL_RX = 0;
	CHNL_RX_LEN = 0;
	CHNL_RX_OFF = 0;
	CHNL_RX_DATA = 0;
	CHNL_RX_DATA_VALID = 0;
	cur_input_idx = 0;
	#(5*`CYCLE);
	@(posedge clk);
	#1;
	CHNL_RX = 1;
	CHNL_RX_LEN = NUM_OF_PAT << 2;
	CHNL_RX_OFF = 0;
	wait(CHNL_RX_ACK);
	
	while ((cur_input_idx < NUM_OF_PAT)) begin
		//pat_length = seq_mem_current[0];
		@(posedge clk);
		#(0.3*`CYCLE);
		k = $random(iters) % 20;
		CHNL_RX_DATA_VALID = (k < 10 & $signed(k) > -10);

		
		/*
			In the testbench, the simulation of the RIFFA TX (Transmission) behavior is carried out. 
			cur_input_idx indicates the index of the current data being transmitted. 
			The index is incremented when certain conditions are met. 
			Typically, what needs to be modified is what data to transmit at a specific index, which corresponds to the data transaction part.
		*/
		// Data transaction part
		CHNL_RX_DATA = golden_in[cur_input_idx];
		////////////////////////

		// Protocol Part
		if (CHNL_RX_DATA_REN) begin
			cur_input_idx = (CHNL_RX_DATA_VALID)? cur_input_idx + 1: cur_input_idx;
		end
		if (~CHNL_RX_DATA_VALID) CHNL_RX_DATA = 0;

		$display("data %d done.",cur_input_idx);
    end
	@(posedge clk);
	#(0.3*`CYCLE);
	CHNL_RX_DATA_VALID = 0;
	CHNL_RX_DATA = 0;
	CHNL_RX = 0;
end


initial begin
	iters = `seed; // initial seed
	k = 0;
end



initial begin
	error = 0;
	cur_output_idx = 0;
	CHNL_TX_ACK = 0;
	CHNL_TX_DATA_REN = 0;
	
	wait(CHNL_TX);
	
	#(1.5*`CYCLE);
	@(posedge clk);
	CHNL_TX_ACK = 1;
	#(`CYCLE);
	CHNL_TX_ACK = 0;

	while (cur_output_idx < NUM_OF_PAT) begin
		@(posedge clk);
		#(0.3*`CYCLE);
		CHNL_TX_DATA_REN = 1;

		if (CHNL_TX_DATA_VALID & CHNL_TX_DATA_REN) begin	// Protocol Part
			
			// Data transaction part
			if (!(golden_out[cur_output_idx] === CHNL_TX_DATA)) begin
				$display(" Pattern %d failed !. Expected candidate = %h, but the Response candidate = %h !! ", cur_output_idx, golden_out[cur_output_idx], CHNL_TX_DATA);
				error = error + 1;
			end 
			
			else begin
				$display("Pattern %d is passed !. Expected candidate = %h, Response candidate = %h !! ", cur_output_idx, golden_out[cur_output_idx], CHNL_TX_DATA);
			end
			///////////////////////

			cur_output_idx = cur_output_idx + 1;
		end
	end

	cur_output_idx = 0;
	
	@(posedge clk);
	#(0.3*`CYCLE);
	CHNL_TX_DATA_REN = 0;
			
	#(`CYCLE*2); 
	if (error == 0) begin
		$display("--------------------------- Simulation Stops !!---------------------------");
		$display("============================================================================");
		$display("\n");
		$display("        ****************************              ");
		$display("        **                        **        /|__/|");
		$display("        **  Congratulations !!    **      / O,O  |");
		$display("        **                        **    /_____   |");
		$display("        **  Simulation Complete!! **   /^ ^ ^ \\  |");
		$display("        **                        **  |^ ^ ^ ^ |w|");
		$display("        *************** ************   \\m___m__|_|");
		$display("\n");
		$display("============================================================================");
	end else begin
		$display("============================================================================");
		$display("\n           FUCK!!! There are %d errors with your code ...!          ",error);
		$display("============================================================================");
	end
	$finish;
end
	
  

  // end of simulation
  initial begin
    #(`CYCLE*`END_CYCLE) // calculate clock cycles for all operation (you can modify it)
		$display("============================================================================");
		$display("\n           Error!!! There is something wrong with your code ...!          ");
		$display("\n                       The test result is .....FAIL                     \n");
		$display("============================================================================");
	 	$finish;
  end
endmodule