//---------------------------------------------------------------------------
// DUT - Mini project
//---------------------------------------------------------------------------
`include "common.vh"

module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    ,
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//input SRAM interface
  output wire                           dut__tb__sram_input_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_input_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_read_address  ,
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_input_read_data     ,

//weight SRAM interface
  output wire                           dut__tb__sram_weight_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_weight_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_read_address  ,
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_weight_read_data     ,

//result SRAM interface
  output wire                           dut__tb__sram_result_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_result_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_read_address  ,
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_result_read_data


);

//============================================
//================CONTROLLER==================
//============================================

// OUTPUTS OF CONTROLLER
wire start, data_start;
reg dut_ready_reg;
reg get_matrix_dim, load_matrix_dim;
reg calculate;

// INPUTS TO CONTROLLER
reg done;
// STATE ASSIGNMENT AND REGISTERS
parameter [8:0]
s0 = 9'b0_0000_0001,
s1 = 9'b0_0000_0010,
s2 = 9'b0_0000_0100,
s3 = 9'b0_0000_1000,
s4 = 9'b0_0001_0000,
s5 = 9'b0_0010_0000,
s6 = 9'b0_0100_0000,
s7 = 9'b0_1000_0000,
s8 = 9'b1_0000_0000;
reg [7:0] current_state, next_state;
reg start_r, data_start_reg;

always @(posedge clk or negedge reset_n) begin
	if (!reset_n) begin
		current_state <= s0;
	end else begin
		current_state <= next_state;
	end
end
always @( posedge clk or negedge reset_n ) begin
	if( !reset_n ) begin
		start_r <= 1'b0;
		data_start_reg <= 1'b0;
	end else begin
		case( current_state)
			s5: begin
				start_r <= 1'b1;
			end
			s7: begin
				if( done ) begin
					start_r <= 1'b0;
					data_start_reg <= 1'b0;
				end else begin
					data_start_reg <= 1'b1;
				end
			end
			s8: begin
				start_r <= 1'b0;
			end
		endcase
	end

end
assign start = start_r;
assign data_start = data_start_reg;

always @(*) begin
	get_matrix_dim = 1'b0;
	load_matrix_dim = 1'b0;
	calculate = 1'b0;
	case (current_state)
		s0: begin
			if (dut_valid) begin
				next_state = s1;
			end else begin
				next_state = s0;
			end
        	end
		s1: begin
			get_matrix_dim = 1'b1;
            		next_state = s2;
        	end
        	s2: begin
            		get_matrix_dim = 1'b0;
	    		next_state = s3;
        	end
        	s3: begin
			load_matrix_dim = 1'b1;
			next_state = s4;
        	end
        	s4: begin
			load_matrix_dim = 1'b0;
	        	calculate = 1'b1;
			next_state = s5;
		end
		s5: begin
			calculate = 1'b0;
			//start = 1'b1;
			next_state = s6;
		end
		s6: begin
	       		next_state = s7;
		end
		s7: begin
                	if ( !done ) begin
                        	next_state = s7;
                	end else begin
				next_state = s8;
                	end
		end
		s8: begin
             		next_state = s0;
		end
        	default: begin
			next_state = s0;
			load_matrix_dim = 1'b0;
        		get_matrix_dim = 1'b0;
        		calculate = 1'b0;
		end
    endcase
end
always @( posedge clk or negedge reset_n ) begin
	if( !reset_n ) begin
		dut_ready_reg <= 1'b1;
	end else if( dut_valid ) begin
		dut_ready_reg <= 1'b0;
	end else if ( done ) begin
		dut_ready_reg <= 1'b1;
	end else begin
		dut_ready_reg <= 1'b0;
	end
end

//endmodule
//==============END CONTROLLER================

//============================================
//=================DATAPATH===================
//============================================
reg write_data;
reg [31:0]  write_address;
reg [`SRAM_ADDR_RANGE] input_read_addr_reg;
reg [`SRAM_ADDR_RANGE] weight_read_addr_reg;
reg [`SRAM_DATA_RANGE]  mac_output_reg;
reg [15:0] input_indexer, weight_indexer, total_accum_count;
reg [31:0] weight_ind_check;
reg [31:0] input_matrix_data, weight_matrix_data;
reg [15:0] input_base_offset;
reg [31:0] total;
reg [31:0] input_matrix_rows, input_matrix_columns, weight_matrix_rows, weight_matrix_columns;
reg [31:0] output_fp;
reg [15:0] accum;
reg [15:0] accum_count;
reg [31:0] accumulate;
reg [7:0] mac_status;
reg data_start_r;

always @( posedge clk or negedge reset_n ) begin
	if( !reset_n ) begin
		total_accum_count <= 16'b0;
	end else if( start && accum == 0 ) begin
		total_accum_count <= total_accum_count + 1'b1;
	end else if( load_matrix_dim ) begin
		total_accum_count <= 16'b0;
	end

end


always @( posedge clk or negedge reset_n ) begin
	if ( !reset_n ) begin
		write_data <= 1'b0;
		data_start_r <= 0;
        	mac_output_reg <= 32'b0;
		accum_count <= 32'b0;
	        done <= 1'b0;
	end else begin
		if( get_matrix_dim ) begin
                        input_read_addr_reg <= 16'b0;
                        weight_read_addr_reg <= 16'b0;
                end
                if( load_matrix_dim ) begin
                        input_matrix_rows <= tb__dut__sram_input_read_data[31:16];
                        input_matrix_columns <= tb__dut__sram_input_read_data[15:0];
                        weight_matrix_rows <= tb__dut__sram_weight_read_data[31:16];
                        weight_matrix_columns <= tb__dut__sram_weight_read_data[15:0];

                        input_indexer <= 16'b1;
                        weight_indexer <= 16'b1;
                        input_base_offset <= 16'b0;
                end
                if( calculate ) begin
                        total <= input_matrix_rows * weight_matrix_columns;
                        accum <= input_matrix_columns;
                        weight_ind_check <= weight_matrix_columns * weight_matrix_rows;
                end
                if( data_start ) begin
                        if( total_accum_count < total ) begin
                                done <= 1'b0;
                        end else begin
                                done <= 1'b1;
                        end
                end
		data_start_r <= data_start;
		if ( data_start ) begin
			if ( accum == 0 ) begin
				write_data <= 1'b1;  // Trigger write operation when done
                        	accum <= input_matrix_columns - 1'b1;
                        	mac_output_reg <= output_fp;
                	end else begin
                        	accum <= accum - 1'b1;
                        	write_data <= 1'b0;
                	end
			input_matrix_data <= tb__dut__sram_input_read_data;
                        weight_matrix_data <= tb__dut__sram_weight_read_data;

		end
		if (start) begin
			if( weight_indexer < weight_ind_check ) begin
                        	weight_indexer <= weight_indexer + 1'b1;
                	end else begin
                        	weight_indexer <= 1'b1;
                	end

			if( accum == 4 ) begin
				accum_count <= accum_count + 1'b1;
                	end

            		if ( accum_count < weight_matrix_columns ) begin
                		if ( input_indexer < input_base_offset + input_matrix_columns ) begin
                    			input_indexer <= input_indexer + 1'b1;
                		end else begin
                    			input_indexer <= input_base_offset + 1'b1;
                		end

            		end else begin
                		accum_count <= 32'b0;
                		input_base_offset <= input_base_offset + input_matrix_columns;
                		input_indexer <= input_base_offset + input_matrix_columns + 1'b1;
            		end
                	input_read_addr_reg <= input_indexer;
                	weight_read_addr_reg <= weight_indexer;
		end
		if( current_state == s7 && done ) begin
			write_data <= 1'b0;
		end
		if( !dut_ready ) begin
			if(write_data) begin
			write_address <= write_address + 1'b1;
			end
		end else begin
                	write_address <= 32'b0;
        	end

		if( dut_valid ) begin
		       	done <= 1'b0;
		end
	end

end
always @( posedge clk or negedge reset_n ) begin
	if( !reset_n ) begin
		accumulate <= 32'b0;
	end else if( dut_valid ) begin
        	accumulate <= 32'b0;
	end else if( accum == 0 && data_start ) begin
		accumulate <= 32'b0;
	end else if( data_start_r ) begin
                accumulate <= output_fp;
        end
end

assign dut_ready = dut_ready_reg;
assign dut__tb__sram_input_write_enable = 1'b0;   // Disable write when reading
assign dut__tb__sram_weight_write_enable = 1'b0;  // Disable write when reading
assign dut__tb__sram_input_read_address = input_read_addr_reg;
assign dut__tb__sram_weight_read_address = weight_read_addr_reg;
assign dut__tb__sram_result_write_enable = write_data;
assign dut__tb__sram_result_write_address = write_address;
assign dut__tb__sram_result_write_data = mac_output_reg;


reg[2:0] inst_rnd;
always@(posedge clk) begin : proc_inst_rnd
        if(!reset_n) inst_rnd <= 0;
        else inst_rnd <= inst_rnd;
end

DW_fp_mac_inst
  FP_MAC (
  .inst_a( input_matrix_data ),
  .inst_b( weight_matrix_data ),
  .inst_c(  accumulate ),
  .inst_rnd( inst_rnd ),
  .z_inst(  output_fp ),
  .status_inst( mac_status )
);

//==============END DATAPATH==================
endmodule

module DW_fp_mac_inst #(
  parameter inst_sig_width = 23,
  parameter inst_exp_width = 8,
  parameter inst_ieee_compliance = 0 // These need to be fixed to decrease error
) (
  input wire [inst_sig_width+inst_exp_width : 0] inst_a,
  input wire [inst_sig_width+inst_exp_width : 0] inst_b,
  input wire [inst_sig_width+inst_exp_width : 0] inst_c,
  input wire [2 : 0] inst_rnd,
  output wire [inst_sig_width+inst_exp_width : 0] z_inst,
  output wire [7 : 0] status_inst
);

  // Instance of DW_fp_mac
  DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
    .a(inst_a),
    .b(inst_b),
    .c(inst_c),
    .rnd(inst_rnd),
    .z(z_inst),
    .status(status_inst)
  );

endmodule: DW_fp_mac_inst