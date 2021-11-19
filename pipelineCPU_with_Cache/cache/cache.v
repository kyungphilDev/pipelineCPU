`include "opcodes.v"

module instruction_cache(clk, reset_n, instruction, cache_data, cache_enable, mem_data, mem_read, mem_address);
	input clk, reset_n;
	input [`WORD_SIZE-1:0] instruction;
	input [`WORD_SIZE-1:0] mem_data;
	
	output reg [`WORD_SIZE-1:0] cache_data;
	output reg mem_read;
	output reg [`WORD_SIZE-1:0] mem_address;

	wire [11:0] tag;
	wire [1:0] idx;
	wire [1:0] bo;

	assign tag = instruction[15:4];
	assign idx = instruction[3:2];
	assign bo = instruction[1:0];

	// [154] LRU-bit {[153:142]tag [141]valid-bit [140:77]4-words} {[76:65]tag [64]valid-bit [63:0]4-words}
	// 2-way set associative instruction cache ( Capacity : 32 words / Line size : 4 words ) 
	reg [`CACHE_LINE_SIZE-1:0] cache[3:0];
	
	// hit whether first cache or second cache among 2-way cache
	wire hit_first, hit_second;
	assign hit_first = ((tag == cache[idx][153:142]) && (cache[idx][141]));
	assign hit_second = ((tag == cache[idx][76:65]) && (cache[idx][64]));

	// if cache miss occurs, cache is diabled while updating.
	output reg cache_enable;
	// update needs 6 cycles.
	reg [3:0] update_count;
	// 4 words should be updated.
	reg [`WORD_SIZE-1:0] update_address[3:0];
	reg [`WORD_SIZE-1:0] update_data[3:0];

	// saved tag and index when missed
	reg [11:0] update_tag;
	reg [1:0] update_idx;

	reg [11:0] tmp_tag;

	always @(posedge(clk)) begin
		if (!reset_n) begin
			cache[0] <= `CACHE_LINE_SIZE'b0;
			cache[1] <= `CACHE_LINE_SIZE'b0;
			cache[2] <= `CACHE_LINE_SIZE'b0;
			cache[3] <= `CACHE_LINE_SIZE'b0;
			
			update_address[0] <= `WORD_SIZE'b0;
			update_address[1] <= `WORD_SIZE'b0;
			update_address[2] <= `WORD_SIZE'b0;
			update_address[3] <= `WORD_SIZE'b0;

			update_data[0] <= `WORD_SIZE'b0;
			update_data[1] <= `WORD_SIZE'b0;
			update_data[2] <= `WORD_SIZE'b0;
			update_data[3] <= `WORD_SIZE'b0;

			cache_data <= 0;
			mem_read <= 0;
			mem_address <= 0;
			cache_enable <= 1;
			update_count <= 0;
			update_tag <= 12'b0;
			update_idx <= 2'b0;
		end
		mem_read <= 0;
		if (!cache_enable) begin
			case(update_count)
				3'b101: begin
					mem_address <= update_address[0];
					mem_read <= 1;
					update_count <= 3'b100;
				end
				3'b100: begin
					mem_address <= update_address[1];
					mem_read <= 1;
					update_data[0] <= mem_data;
					update_count <= 3'b011;
				end
				3'b011: begin
					mem_address <= update_address[2];
					mem_read <= 1;
					update_data[1] <= mem_data;
					update_count <= 3'b010;
				end
				3'b010: begin
					mem_address <= update_address[3];
					mem_read <= 1;
					update_data[2] <= mem_data;
					update_count <= 3'b001;
				end
				3'b001: begin
					update_data[3] <= mem_data;
					update_count <= 3'b000;
				end
				3'b000: begin
					if (!cache[update_idx][141]) begin
						cache[update_idx][153:142] <= update_tag;
						cache[update_idx][141] <= 1;
						cache[update_idx][140:125] <= update_data[0];
						cache[update_idx][124:109] <= update_data[1];
						cache[update_idx][108:93] <= update_data[2];
						cache[update_idx][92:77] <= update_data[3];
					end
					else if (!cache[update_idx][64]) begin

						cache[update_idx][76:65] <= update_tag;
						cache[update_idx][64] <= 1;
						cache[update_idx][63:48] <= update_data[0];
						cache[update_idx][47:32] <= update_data[1];
						cache[update_idx][31:16] <= update_data[2];
						cache[update_idx][15:0] <= update_data[3];
					end
					else if (cache[update_idx][154]) begin
						cache[update_idx][76:65] <= update_tag;
						cache[update_idx][64] <= 1;
						cache[update_idx][63:48] <= update_data[0];
						cache[update_idx][47:32] <= update_data[1];
						cache[update_idx][31:16] <= update_data[2];
						cache[update_idx][15:0] <= update_data[3];
					end
					else begin
						cache[update_idx][153:142] <= update_tag;
						cache[update_idx][141] <= 1;
						cache[update_idx][140:125] <= update_data[0];
						cache[update_idx][124:109] <= update_data[1];
						cache[update_idx][108:93] <= update_data[2];
						cache[update_idx][92:77] <= update_data[3];
					end
					cache_enable = 1;
				end
			endcase
		end
	end

	always @(negedge(clk)) begin
		if (cache_enable) begin
			if (hit_first) begin
				case(bo)
					2'b00: cache_data <= cache[idx][140:125];
					2'b01: cache_data <= cache[idx][124:109];
					2'b10: cache_data <= cache[idx][108:93];
					2'b11: cache_data <= cache[idx][92:77];
				endcase
				cache[idx][154] <= 1;
			end
			else if (hit_second) begin
				case(bo)
					2'b00: cache_data <= cache[idx][63:48];
					2'b01: cache_data <= cache[idx][47:32];
					2'b10: cache_data <= cache[idx][31:16];
					2'b11: cache_data <= cache[idx][15:0];
				endcase
				cache[idx][154] <= 0;
			end
			else begin
				// cache miss occured, update is needed.
				cache_enable <= 0;
				update_count <= 5;
				update_tag <= tag;
				update_idx <= idx;
				update_address[0] <= {instruction[15:2],2'b00};
				update_address[1] <= {instruction[15:2],2'b01};
				update_address[2] <= {instruction[15:2],2'b10};
				update_address[3] <= {instruction[15:2],2'b11};
			end
		end
	end
endmodule

module data_cache(clk, reset_n, data_address, write_data, cache_read, cache_write, cache_data, cache_enable, mem_data, mem_read, mem_address);
	input clk, reset_n;
	input [`WORD_SIZE-1:0] data_address;
	// updated result to write data
	input [`WORD_SIZE-1:0] write_data;
	// read data from memory
	input [`WORD_SIZE-1:0] mem_data;
	input cache_read, cache_write;

	output reg [`WORD_SIZE-1:0] cache_data;
	output reg mem_read;
	output reg [`WORD_SIZE-1:0] mem_address;

	wire [11:0] tag;
	wire [1:0] idx;
	wire [1:0] bo;

	assign tag = data_address[15:4];
	assign idx = data_address[3:2];
	assign bo = data_address[1:0];

  	assign data2 = (cache_write) ? write_data : 16'bz;

	// [154] LRU-bit {[153:142]tag [141]valid-bit [140:77]4-words} {[76:65]tag [64]valid-bit [63:0]4-words}
	// 2-way set associative instruction cache ( Capacity : 32 words / Line size : 4 words ) 
	reg [`CACHE_LINE_SIZE-1:0] cache[3:0];
	
	// hit whether first cache or second cache among 2-way cache
	wire hit_first, hit_second;
	assign hit_first = ((tag == cache[idx][153:142]) && (cache[idx][141]));
	assign hit_second = ((tag == cache[idx][76:65]) && (cache[idx][64]));

	// if cache miss occurs, cache is diabled while updating.
	output reg cache_enable;
	// update needs 6 cycles.
	reg [3:0] update_count;
	// 4 words should be updated.
	reg [`WORD_SIZE-1:0] update_address[3:0];
	reg [`WORD_SIZE-1:0] update_data[3:0];

	// saved tag and index when missed
	reg [11:0] update_tag;
	reg [1:0] update_idx;
	// when write to cache case, only delay is needed.
	reg write_case;

	always @(posedge(clk)) begin
		if (!reset_n) begin
			cache[0] <= `CACHE_LINE_SIZE'b0;
			cache[1] <= `CACHE_LINE_SIZE'b0;
			cache[2] <= `CACHE_LINE_SIZE'b0;
			cache[3] <= `CACHE_LINE_SIZE'b0;
			
			update_address[0] <= `WORD_SIZE'b0;
			update_address[1] <= `WORD_SIZE'b0;
			update_address[2] <= `WORD_SIZE'b0;
			update_address[3] <= `WORD_SIZE'b0;

			update_data[0] <= `WORD_SIZE'b0;
			update_data[1] <= `WORD_SIZE'b0;
			update_data[2] <= `WORD_SIZE'b0;
			update_data[3] <= `WORD_SIZE'b0;

			cache_data <= 0;
			mem_read <= 0;
			mem_address <= 0;
			cache_enable <= 1;
			update_count <= 0;
			update_tag <= 12'b0;
			update_idx <= 2'b0;
		end
		mem_read <= 0;
		if (!write_case && !cache_enable) begin
			case(update_count)
				3'b101: begin
					mem_address <= update_address[0];
					mem_read <= 1;
					update_count <= 3'b100;
				end
				3'b100: begin
					mem_address <= update_address[1];
					mem_read <= 1;
					update_data[0] <= mem_data;
					update_count <= 3'b011;
				end
				3'b011: begin
					mem_address <= update_address[2];
					mem_read <= 1;
					update_data[1] <= mem_data;
					update_count <= 3'b010;
				end
				3'b010: begin
					mem_address <= update_address[3];
					mem_read <= 1;
					update_data[2] <= mem_data;
					update_count <= 3'b001;
				end
				3'b001: begin
					update_data[3] <= mem_data;
					update_count <= 3'b000;
				end
				3'b000: begin
					if (!cache[update_idx][141]) begin
						cache[update_idx][153:142] <= update_tag;
						cache[update_idx][141] <= 1;
						cache[update_idx][140:125] <= update_data[0];
						cache[update_idx][124:109] <= update_data[1];
						cache[update_idx][108:93] <= update_data[2];
						cache[update_idx][92:77] <= update_data[3];
					end
					else if (!cache[update_idx][64]) begin

						cache[update_idx][76:65] <= update_tag;
						cache[update_idx][64] <= 1;
						cache[update_idx][63:48] <= update_data[0];
						cache[update_idx][47:32] <= update_data[1];
						cache[update_idx][31:16] <= update_data[2];
						cache[update_idx][15:0] <= update_data[3];
					end
					else if (cache[update_idx][154]) begin
						cache[update_idx][76:65] <= update_tag;
						cache[update_idx][64] <= 1;
						cache[update_idx][63:48] <= update_data[0];
						cache[update_idx][47:32] <= update_data[1];
						cache[update_idx][31:16] <= update_data[2];
						cache[update_idx][15:0] <= update_data[3];
					end
					else begin
						cache[update_idx][153:142] <= update_tag;
						cache[update_idx][141] <= 1;
						cache[update_idx][140:125] <= update_data[0];
						cache[update_idx][124:109] <= update_data[1];
						cache[update_idx][108:93] <= update_data[2];
						cache[update_idx][92:77] <= update_data[3];
					end
					cache_enable <= 1;
				end
			endcase
		end
		// data cache write case
		else if (write_case && !cache_enable) begin
			case(update_count)
				3'b101: begin
					update_count <= 3'b100;
				end
				3'b100: begin
					update_count <= 3'b011;
				end
				3'b011: begin
					update_count <= 3'b010;
				end
				3'b010: begin
					update_count <= 3'b001;
				end
				3'b001: begin
					update_count <= 3'b000;
				end
				3'b000: begin
					cache_enable <= 1;
					write_case <= 0;
				end
			endcase
		end
	end

	always @(negedge(clk)) begin
		if (cache_read && cache_enable) begin
			if (hit_first) begin
				case(bo)
					2'b00: cache_data <= cache[idx][140:125];
					2'b01: cache_data <= cache[idx][124:109];
					2'b10: cache_data <= cache[idx][108:93];
					2'b11: cache_data <= cache[idx][92:77];
				endcase
				cache[update_idx][154] <= 1;
			end
			else if (hit_second) begin
				case(bo)
					2'b00: cache_data <= cache[idx][63:48];
					2'b01: cache_data <= cache[idx][47:32];
					2'b10: cache_data <= cache[idx][31:16];
					2'b11: cache_data <= cache[idx][15:0];
				endcase
				cache[update_idx][154] <= 0;
			end
			else begin
				// cache miss occured, update is needed.
				cache_enable <= 0;
				update_count <= 5;
				update_tag <= tag;
				update_idx <= idx;
				update_address[0] <= {data_address[15:2],2'b00};
				update_address[1] <= {data_address[15:2],2'b01};
				update_address[2] <= {data_address[15:2],2'b10};
				update_address[3] <= {data_address[15:2],2'b11};
			end
		end
		else if (cache_write && cache_enable) begin
			if (hit_first) begin
				case(bo)
					2'b00: cache[idx][140:125] <= write_data;
					2'b01: cache[idx][124:109] <= write_data;
					2'b10: cache[idx][108:93] <= write_data;
					2'b11: cache[idx][92:77] <= write_data;
				endcase
				cache[update_idx][154] <= 1;
			end
			else if (hit_second) begin
				case(bo)
					2'b00: cache[idx][63:48] <= write_data;
					2'b01: cache[idx][47:32] <= write_data;
					2'b10: cache[idx][31:16] <= write_data;
					2'b11: cache[idx][15:0] <= write_data;
				endcase
				cache[update_idx][154] <= 0;
			end
			// change in data inside cache occured, delay is needed.
			cache_enable <= 0;
			update_count <= 5;

			write_case <= 1;
		end
	end
endmodule